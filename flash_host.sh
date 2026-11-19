#!/bin/sh
# Flash the Yesoul_BLE firmware to an ESP32-C3 connected on the HOST.
#
# Why this script exists: the VS Code Flatpak sandbox cannot open /dev/ttyACM*
# (the sandbox maps the node owner to the unprivileged "nfsnobody" group, so the
# kernel's permission check against the real "dialout" group always fails, and a
# chmod from inside the sandbox is reverted each time the port re-enumerates).
# The host, however, can run the same PlatformIO esptool binary directly.
#
# Usage (from a host terminal):
#   sudo sh /home/waterfallm/GitHub/HomeLab/Yesoul_BLE/flash_host.sh [PORT]
set -e

PORT="${1:-/dev/ttyACM0}"
BUILD=/home/waterfallm/GitHub/HomeLab/Yesoul_BLE/.pio/build/esp32-c3
ESPTOOL=/home/waterfallm/.platformio/packages/tool-esptool/esptool
BOOT_APP0=/home/waterfallm/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin

echo "Target port : $PORT"
ls -l "$PORT"

# Mirrors exactly the command PlatformIO generates for env:esp32-c3.
exec "$ESPTOOL" \
  --chip esp32c3 \
  --port "$PORT" \
  --baud 460800 \
  --before default-reset \
  --after hard-reset \
  write-flash -z \
  --flash-mode dio \
  --flash-freq 80m \
  --flash-size 4MB \
  0x0000  "$BUILD/bootloader.bin" \
  0x8000  "$BUILD/partitions.bin" \
  0xe000  "$BOOT_APP0" \
  0x10000 "$BUILD/firmware.bin"
