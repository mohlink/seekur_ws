#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

class SeekurBaudrateDetector {
private:
    std::vector<int> supported_baudrates_ = {9600, 19200, 38400, 57600, 115200};
    
    bool testBaudrate(const std::string& port, int baudrate) {
        std::cout << "Test baudrate " << baudrate << "... ";
        
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1) {
            std::cout << "ERREUR ouverture port" << std::endl;
            return false;
        }
        
        // Configuration port série
        struct termios options;
        tcgetattr(fd, &options);
        
        speed_t speed = B9600;
        switch(baudrate) {
            case 9600:   speed = B9600;   break;
            case 19200:  speed = B19200;  break;
            case 38400:  speed = B38400;  break;
            case 57600:  speed = B57600;  break;
            case 115200: speed = B115200; break;
        }
        
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);
        
        // Configuration 8N1
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag |= CREAD | CLOCAL;
        
        // Mode raw
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        
        // Timeout court pour les tests
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10; // 1 seconde
        
        tcsetattr(fd, TCSANOW, &options);
        
        // Test SYNC0
        std::vector<uint8_t> sync0 = {0xFA, 0xFB, 0x03, 0x00, 0x00, 0x00};
        ssize_t written = write(fd, sync0.data(), sync0.size());
        
        if (written != static_cast<ssize_t>(sync0.size())) {
            std::cout << "ECHEC écriture" << std::endl;
            close(fd);
            return false;
        }
        
        // Attente réponse
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        uint8_t buffer[64];
        ssize_t bytes_read = read(fd, buffer, sizeof(buffer));
        
        close(fd);
        
        // Vérification si on reçoit un écho du SYNC0
        if (bytes_read >= 6) {
            bool echo_found = false;
            for (int i = 0; i <= bytes_read - 6; i++) {
                if (buffer[i] == 0xFA && buffer[i+1] == 0xFB && 
                    buffer[i+2] == 0x03 && buffer[i+3] == 0x00) {
                    echo_found = true;
                    break;
                }
            }
            
            if (echo_found) {
                std::cout << "SUCCES (écho reçu)" << std::endl;
                return true;
            } else {
                std::cout << "Données reçues mais pas d'écho SYNC" << std::endl;
                // Affichage des données pour debug
                std::cout << "  Reçu " << bytes_read << " bytes: ";
                for (int i = 0; i < bytes_read && i < 20; i++) {
                    printf("0x%02X ", buffer[i]);
                }
                std::cout << std::endl;
                return false;
            }
        } else {
            std::cout << "ECHEC (pas de réponse)" << std::endl;
            return false;
        }
    }
    
public:
    int detectBaudrate(const std::string& port) {
        std::cout << "=== Détection Baudrate Seekur Jr ===" << std::endl;
        std::cout << "Port: " << port << std::endl;
        std::cout << "Baudrates testés selon documentation SeekurOS:" << std::endl;
        
        for (int baudrate : supported_baudrates_) {
            if (testBaudrate(port, baudrate)) {
                std::cout << "\n✓ Baudrate détecté: " << baudrate << std::endl;
                return baudrate;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        std::cout << "\n✗ Aucun baudrate fonctionnel détecté" << std::endl;
        std::cout << "Vérifiez:" << std::endl;
        std::cout << "- Le robot est allumé" << std::endl;
        std::cout << "- Les câbles sont connectés" << std::endl;
        std::cout << "- Le port série est correct" << std::endl;
        std::cout << "- Aucun autre programme n'utilise le port" << std::endl;
        
        return -1;
    }
    
    void scanPorts() {
        std::cout << "=== Scan des Ports Série ===" << std::endl;
        
        std::vector<std::string> possible_ports = {
            "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3",
            "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3",
            "/dev/ttyACM0", "/dev/ttyACM1"
        };
        
        std::vector<std::string> available_ports;
        
        for (const auto& port : possible_ports) {
            int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            if (fd != -1) {
                available_ports.push_back(port);
                close(fd);
                std::cout << "✓ " << port << " - disponible" << std::endl;
            } else {
                std::cout << "✗ " << port << " - indisponible" << std::endl;
            }
        }
        
        std::cout << "\nPorts disponibles: " << available_ports.size() << std::endl;
        
        // Test automatique sur les ports disponibles
        for (const auto& port : available_ports) {
            std::cout << "\n--- Test port " << port << " ---" << std::endl;
            int baudrate = detectBaudrate(port);
            if (baudrate > 0) {
                std::cout << "\n🎯 TROUVE: " << port << " @ " << baudrate << " bauds" << std::endl;
                std::cout << "Commande pour utiliser:" << std::endl;
                std::cout << "./seekur_test " << port << " " << baudrate << std::endl;
                return;
            }
        }
        
        std::cout << "\n❌ Aucun Seekur Jr détecté sur les ports testés" << std::endl;
    }
};

int main(int argc, char* argv[]) {
    SeekurBaudrateDetector detector;
    
    if (argc > 1) {
        // Test d'un port spécifique
        std::string port = argv[1];
        int result = detector.detectBaudrate(port);
        return (result > 0) ? 0 : 1;
    } else {
        // Scan automatique
        detector.scanPorts();
        return 0;
    }
}