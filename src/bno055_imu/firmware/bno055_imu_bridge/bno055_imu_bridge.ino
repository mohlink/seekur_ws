/*
 * bno055_imu_bridge.ino — Pont série BNO055 -> ROS2 (robot_localization EKF)
 * Projet : SeekurJR / Navigation autonome (Volet B)
 *
 * Choix de conception :
 *   - Mode IMUPLUS (accel + gyro, SANS magnétomètre) : le magnéto n'est pas
 *     fiable en mine (roche ferreuse, châssis acier, moteurs). Yaw relatif.
 *   - Entiers bruts du BNO055 (aucune perte de précision, aucun float sur AVR).
 *     La conversion en unités SI est faite côté nœud ROS2.
 *   - Accéléro brut AVEC gravité (REP-145 / robot_localization).
 *   - 115200 baud, 100 Hz par défaut, numéro de séquence + checksum XOR.
 *   - Offsets de calibration sauvegardés en EEPROM et rechargés au boot.
 *
 * Trame de données (une ligne par échantillon) :
 *   I,seq,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,cal*HH\r\n
 *     seq      : compteur 0..65535 (détection de trames perdues)
 *     q*       : quaternion,        1 LSB = 1/16384
 *     g*       : vitesse angulaire, 1 LSB = 1/16 deg/s   (-> rad/s côté ROS)
 *     a*       : accélération brute (gravité incluse), 1 LSB = 0.01 m/s²
 *     cal      : octet CALIB_STAT (bits 7-6 sys, 5-4 gyro, 3-2 accel, 1-0 mag)
 *     HH       : XOR de tous les caractères entre 'I' et '*' inclus, en hexa
 *   Les lignes commençant par '#' sont des messages d'information (à ignorer).
 *
 * Commandes (envoyer un caractère sur le port série) :
 *   s : sauvegarder les offsets en EEPROM (exige gyro=3 et accel=3)
 *   c : effacer les offsets en EEPROM
 *   i : afficher l'état (calibration, offsets chargés, fréquence)
 *
 * Dépendances : Adafruit BNO055, Adafruit Unified Sensor (gestionnaire de bibliothèques)
 */

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ---------------------------------------------------------------- Réglages
#define SERIAL_BAUD       115200
#define RATE_HZ           100        // 100 max (fréquence de fusion interne du BNO055)
#define BNO_ADDR          0x28       // 0x29 si la broche ADR est à l'état haut
#define USE_EXT_CRYSTAL   true       // false si la carte n'a pas de quartz 32 kHz

// ------------------------------------------------------- Registres BNO055
#define REG_ACC_DATA      0x08       // 0x08..0x19 : ACC(6) MAG(6) GYR(6)
#define REG_QUA_DATA      0x20       // 0x20..0x35 : QUA(8) LIA(6) GRV(6) TEMP(1) CALIB(1)
#define REG_UNIT_SEL      0x3B

// ------------------------------------------------------------- EEPROM
const uint32_t CAL_MAGIC = 0xB055CA11;
struct CalStore {
  uint32_t magic;
  adafruit_bno055_offsets_t offsets;
};

Adafruit_BNO055 bno(55, BNO_ADDR, &Wire);

const uint32_t PERIOD_US = 1000000UL / RATE_HZ;
uint32_t nextTick = 0;
uint16_t seq = 0;
bool offsetsLoaded = false;
uint32_t i2cErrors = 0;
uint32_t lastWarnMs = 0;

// Statistiques de fréquence réelle
uint32_t statWindowStart = 0;
uint16_t statFrames = 0;
float measuredHz = 0.0f;

// ------------------------------------------------------------ EEPROM I/O
void eepromBegin() {
#if defined(ESP32) || defined(ESP8266)
  EEPROM.begin(sizeof(CalStore));
#endif
}

void eepromCommit() {
#if defined(ESP32) || defined(ESP8266)
  EEPROM.commit();
#endif
}

bool loadOffsets() {
  CalStore st;
  EEPROM.get(0, st);
  if (st.magic != CAL_MAGIC) return false;
  bno.setSensorOffsets(st.offsets);   // bascule en CONFIG puis restaure le mode
  return true;
}

void saveOffsets() {
  uint8_t sys, gyr, acc, mag;
  bno.getCalibration(&sys, &gyr, &acc, &mag);
  if (gyr < 3 || acc < 3) {
    Serial.print(F("# SAVE refuse : gyro="));
    Serial.print(gyr);
    Serial.print(F(" accel="));
    Serial.print(acc);
    Serial.println(F(" (les deux doivent valoir 3)"));
    return;
  }
  CalStore st;
  st.magic = CAL_MAGIC;
  bno.getSensorOffsets(st.offsets);  // interrompt brièvement la fusion
  EEPROM.put(0, st);
  eepromCommit();
  offsetsLoaded = true;
  Serial.println(F("# SAVE ok : offsets enregistres en EEPROM"));
}

void clearOffsets() {
  CalStore st;
  memset(&st, 0, sizeof(st));
  EEPROM.put(0, st);
  eepromCommit();
  offsetsLoaded = false;
  Serial.println(F("# CLEAR ok : offsets effaces (effectif au prochain boot)"));
}

// ------------------------------------------------------------ I2C brut
bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)BNO_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

inline int16_t le16(const uint8_t *b) {
  return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

// ------------------------------------------------------------ Init capteur
bool initSensor() {
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) return false;
  delay(20);
  bno.setExtCrystalUse(USE_EXT_CRYSTAL);

  // Forcer les unités (m/s², deg/s, degrés, °C) : garantit les facteurs d'échelle
  bno.setMode(OPERATION_MODE_CONFIG);
  delay(25);
  writeReg(REG_UNIT_SEL, 0x00);
  bno.setMode(OPERATION_MODE_IMUPLUS);
  delay(25);

  offsetsLoaded = loadOffsets();
  return true;
}

void printStatus() {
  uint8_t sys, gyr, acc, mag;
  bno.getCalibration(&sys, &gyr, &acc, &mag);
  Serial.print(F("# STATUS mode=IMUPLUS cal sys/gyr/acc/mag="));
  Serial.print(sys); Serial.print('/');
  Serial.print(gyr); Serial.print('/');
  Serial.print(acc); Serial.print('/');
  Serial.print(mag);
  Serial.print(F(" offsets_eeprom="));
  Serial.print(offsetsLoaded ? F("oui") : F("non"));
  Serial.print(F(" hz="));
  Serial.print(measuredHz, 1);
  Serial.print(F(" i2c_err="));
  Serial.println(i2cErrors);
}

void handleCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 's') saveOffsets();
    else if (c == 'c') clearOffsets();
    else if (c == 'i') printStatus();
  }
}

// ------------------------------------------------------------ Setup / loop
void setup() {
  Serial.begin(SERIAL_BAUD);
  eepromBegin();
  Wire.begin();   // 100 kHz par défaut : suffisant, et plus sûr avec le clock stretching du BNO055

  Serial.println(F("# bno055_imu_bridge v1.0"));
  while (!initSensor()) {
    Serial.println(F("# ERR BNO055 introuvable, nouvelle tentative dans 1 s"));
    delay(1000);
  }
  Serial.print(F("# READY rate_hz="));
  Serial.print(RATE_HZ);
  Serial.print(F(" offsets_eeprom="));
  Serial.println(offsetsLoaded ? F("oui") : F("non"));

  nextTick = micros();
  statWindowStart = millis();
}

void loop() {
  handleCommands();

  uint32_t now = micros();
  if ((int32_t)(now - nextTick) < 0) return;
  nextTick += PERIOD_US;
  if ((int32_t)(now - nextTick) > (int32_t)PERIOD_US) nextTick = now + PERIOD_US; // retard : resynchronisation

  uint8_t a[18];   // ACC, MAG, GYR
  uint8_t q[22];   // QUA, LIA, GRV, TEMP, CALIB
  if (!readRegs(REG_ACC_DATA, a, sizeof(a)) || !readRegs(REG_QUA_DATA, q, sizeof(q))) {
    i2cErrors++;
    if (millis() - lastWarnMs > 1000) {
      Serial.print(F("# WARN lecture I2C echouee, total="));
      Serial.println(i2cErrors);
      lastWarnMs = millis();
    }
    return;
  }

  int16_t ax = le16(&a[0]),  ay = le16(&a[2]),  az = le16(&a[4]);
  int16_t gx = le16(&a[12]), gy = le16(&a[14]), gz = le16(&a[16]);
  int16_t qw = le16(&q[0]),  qx = le16(&q[2]),  qy = le16(&q[4]), qz = le16(&q[6]);
  uint8_t cal = q[21];

  char line[96];
  int n = snprintf(line, sizeof(line), "I,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u",
                   seq, qw, qx, qy, qz, gx, gy, gz, ax, ay, az, cal);
  uint8_t cs = 0;
  for (int i = 0; i < n; i++) cs ^= (uint8_t)line[i];

  Serial.print(line);
  Serial.print('*');
  if (cs < 0x10) Serial.print('0');
  Serial.println(cs, HEX);

  seq++;

  // Fréquence réelle mesurée sur des fenêtres de 1 s
  statFrames++;
  uint32_t ms = millis();
  if (ms - statWindowStart >= 1000) {
    measuredHz = statFrames * 1000.0f / (ms - statWindowStart);
    statFrames = 0;
    statWindowStart = ms;
  }
}
