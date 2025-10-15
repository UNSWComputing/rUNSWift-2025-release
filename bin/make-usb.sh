set -e
set -o pipefail
REALPATH="$(realpath "$0")"
BIN_DIR="$(dirname "$REALPATH")"

IMAGE_FILE="$BIN_DIR/../firmware/nao-2.8.5.11-ubuntu-22.04-ros2.opn"

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <usb device>"
    echo "Example: $0 /dev/sda"
    echo "Find USB device on Mac https://osxdaily.com/2015/06/05/copy-iso-to-usb-drive-mac-os-x-command/"
    echo "Find USB device on Linux https://itsfoss.com/list-usb-devices-linux/"
    exit 1
fi

OUTPUT_DEVICE="$1"
if [ ! -b "$OUTPUT_DEVICE" ]; then
  echo "Error: $OUTPUT_DEVICE is not a valid block device"
  exit 1
fi

# parameters
echo "Writing $IMAGE_FILE to $1"
sudo dd if=$IMAGE_FILE of=$OUTPUT_DEVICE bs=4096
