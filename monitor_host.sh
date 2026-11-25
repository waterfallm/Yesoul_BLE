#!/bin/sh
# Watch the firmware's diagnostic output on the HOST.
#
# Run this from a host terminal (not inside the VS Code Flatpak sandbox, which
# cannot open /dev/ttyACM*).
#
# Usage:
#   sh /home/waterfallm/GitHub/HomeLab/Yesoul_BLE/monitor_host.sh [PORT]
#
# Note on permissions: your host user is currently NOT in the `dialout` group,
# so the port can only be opened as root. The recommended permanent fix (avoids
# typing sudo repeatedly, and is safer than pasting passwords):
#
#   sudo usermod -aG dialout $USER      # then log out and back in
#
# This script uses sudo automatically only if the plain open fails.
#
# What you should see:
#   Starting NimBLE Server
#   Advertising Started
#   BLE Advertised Device found: ...        <- each nearby BLE device
#   Detected bike: G1M Plus (name=YS_G1MPLUS..) powerScale=1.000
#   Forming a connection to <bike MAC>
#   We are now connected to the BLE Server.
#   Power = 120 | Cadence = 85 | Speed = 24.30 km/h | Resistance = 12
#
# The final line repeats while pedalling and is the primary thing to verify
# against what the bike's own display shows.
#
# Press Ctrl-] to quit.
set -e

PORT="${1:-/dev/ttyACM0}"
PIO=/home/waterfallm/GitHub/HomeLab/Yesoul_BLE/.pio_venv/bin/pio

if [ ! -x "$PIO" ]; then
  echo "PlatformIO venv not found at $PIO" >&2
  echo "Recreate it with:" >&2
  echo "  python3 -m venv /home/waterfallm/GitHub/HomeLab/Yesoul_BLE/.pio_venv" >&2
  echo "  /home/waterfallm/GitHub/HomeLab/Yesoul_BLE/.pio_venv/bin/pip install platformio" >&2
  exit 1
fi

echo "Monitoring $PORT at 115200 baud. Press Ctrl-] to quit."

# Use sudo only if the port is not openable as the current user. sudo will
# prompt for your password interactively; it is never stored by this script.
if python3 -c "open('$PORT', 'rb').close()" 2>/dev/null; then
  exec "$PIO" device monitor --port "$PORT" --baud 115200
else
  echo "Cannot open $PORT as $(id -un); retrying with sudo (you will be prompted)." >&2
  exec sudo "$PIO" device monitor --port "$PORT" --baud 115200
fi

