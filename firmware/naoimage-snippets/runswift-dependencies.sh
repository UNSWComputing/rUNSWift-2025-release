mount -o bind /dev ./root/dev
mount -o bind /dev/pts ./root/dev/pts
mount -t sysfs /sys ./root/sys
mount -t proc /proc ./root/proc

chroot ./root /bin/bash <<"EOT"
set -e
set -o pipefail

export DEBIAN_FRONTEND=noninteractive

# First, remove unnecessary packages to free up some much needed space
sudo apt remove --purge -y mesa-vulkan-drivers python3-scipy gfortran-11
sudo apt -y autoremove
sudo apt clean

# Then it's possible to install more packages
apt-get install -y flite libprotobuf23 python-is-python3 net-tools rsync libjpeg-dev
apt-get install -y libboost-filesystem1.74.0 libboost-thread1.74.0 libboost-program-options1.74.0 libboost-serialization1.74.0 libboost-python1.74.0 python3-pip
apt-get install -y libasound2-dev
apt-get install -y libopencv-dev
apt-get install -y python3-construct

# Install the dependencies for comms/audio_messsages
apt-get install -y portaudio19-dev 
apt-get install -y python3-pyaudio

# Ideally this would be in requirements.txt
pip3 install pyalsaaudio==0.10.0 numpy==1.26.4 msgpack==1.0.8
pip3 install onnxruntime==1.20.1
pip3 install xacro==1.13.3

pip3 install transforms3d==0.3.1
EOT

umount ./root/dev/pts
umount ./root/dev
umount ./root/sys
umount ./root/proc
