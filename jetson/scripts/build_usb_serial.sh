#!/usr/bin/env bash
# build_usb_serial.sh — compile et installe ch341 et pl2303 pour le noyau courant.
#
# Pourquoi : le noyau JetPack 7.2.1 (6.8.12-1021-tegra) n'inclut ni ch341
# (IMU Nano CH340) ni pl2303 (adaptateur série du SeekurJR). Seuls cp210x,
# ftdi_sio, option et garmin_gps sont fournis, et il n'existe pas de paquet
# linux-modules-extra pour ce noyau.
#
# Les sources (code noyau tiers, GPL) ne sont PAS versionnées : elles sont
# téléchargées depuis l'arbre stable correspondant à la version du noyau.
#
# Usage : ./build_usb_serial.sh            (noyau courant)
#         KREL=<version> ./build_usb_serial.sh   (autre noyau installé)
#
# À relancer après toute mise à jour de nvidia-l4t-kernel (voir jetson/README.md).
set -euo pipefail

KREL="${KREL:-$(uname -r)}"                    # ex. 6.8.12-1021-tegra
KVER="${KVER:-${KREL%%-*}}"                    # ex. 6.8.12 -> tag v6.8.12
BUILD_DIR="${BUILD_DIR:-$HOME/kmod/usb-serial}"
KDIR="/lib/modules/${KREL}/build"
DEST="/lib/modules/${KREL}/updates/usb-serial"
BASE="https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/usb/serial"

echo "== Noyau cible : ${KREL} (sources v${KVER})"

if [ ! -d "${KDIR}" ]; then
  echo "ERREUR : en-têtes absents (${KDIR}). Installer nvidia-l4t-kernel-headers." >&2
  exit 1
fi

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

echo "== Téléchargement des sources"
for f in ch341.c pl2303.c pl2303.h; do
  curl -fsSL -o "${f}" "${BASE}/${f}?h=v${KVER}"
  echo "   ${f} : $(wc -l < "${f}") lignes"
done

echo 'obj-m := ch341.o pl2303.o' > Kbuild

echo "== Compilation"
make -C "${KDIR}" M="${BUILD_DIR}" clean >/dev/null
make -C "${KDIR}" M="${BUILD_DIR}" modules

echo "== Installation dans ${DEST}"
sudo install -D -m 644 ch341.ko  "${DEST}/ch341.ko"
sudo install -D -m 644 pl2303.ko "${DEST}/pl2303.ko"
sudo depmod -a "${KREL}"

if [ "${KREL}" = "$(uname -r)" ]; then
  echo "== Chargement"
  sudo modprobe -a ch341 pl2303
  lsmod | grep -E "^(ch341|pl2303)" || true
else
  echo "== Noyau ${KREL} non actif : modules chargés automatiquement au prochain démarrage."
fi

echo "== OK. L'avertissement 'tainting kernel' dans dmesg est normal (modules non signés)."
