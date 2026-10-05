#!/bin/sh
# ota_push.sh - build an ESP32 panel sketch and send it to the panel over Wi-Fi.
#
#   ./ota_push.sh chladni5                 build, push to alchemy-panel.local
#   ./ota_push.sh chladni5 192.168.1.50    push to an address instead
#
# The sketch needs wifi_ota.h and a wifi_secrets.h (see chladni5/).  The panel
# reboots into the new image; if that image cannot get back onto Wi-Fi
# within 90 s it restarts and the bootloader rolls back to the previous one.
# The panel's own page, http://alchemy-panel.local/, says which image runs.
cd "$(dirname "$0")" || exit 1
set -e

SKETCH=${1:?usage: $0 <sketch folder> [host]}
SKETCH=${SKETCH%/}
HOST=${2:-alchemy-panel.local}
CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
FQBN="esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc"
OUT="${TMPDIR:-/tmp}/panel-build-$SKETCH"

if [ ! -f "$SKETCH/wifi_secrets.h" ]; then
  echo "$SKETCH/wifi_secrets.h is missing - an image without it has no Wi-Fi," >&2
  echo "so it could never be updated this way again (it would roll back)." >&2
  exit 1
fi

# Stop on a failed build: the old pipe through grep hid the compiler's exit
# status, so a broken sketch pushed whatever image was left in $OUT.
rm -f "$OUT/$SKETCH.ino.bin"
if ! "$CLI" compile --fqbn "$FQBN" --output-dir "$OUT" "$SKETCH" > "$OUT.log" 2>&1; then
  grep -E "error" "$OUT.log" >&2
  echo "build failed - nothing pushed (full log: $OUT.log)" >&2
  exit 1
fi
grep -E "Sketch uses" "$OUT.log" || true
echo "pushing $OUT/$SKETCH.ino.bin to $HOST ..."
curl -sS --max-time 300 -F "firmware=@$OUT/$SKETCH.ino.bin" "http://$HOST/update"
