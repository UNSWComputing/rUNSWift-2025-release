mount -o bind /dev ./root/dev
mount -o bind /dev/pts ./root/dev/pts
mount -t sysfs /sys ./root/sys
mount -t proc /proc ./root/proc

chroot ./root /bin/bash <<"EOT"
set -e
set -o pipefail

export DEBIAN_FRONTEND=noninteractive

sudo apt install -y ros-humble-usb-cam ros-humble-vision-msgs ros-humble-tf-transformations v4l-utils ros-humble-imu-filter-madgwick
sudo apt install -y ros-humble-foxglove-bridge
EOT

umount ./root/dev/pts
umount ./root/dev
umount ./root/sys
umount ./root/proc
