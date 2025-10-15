############################ START FRAMEWORK INSTALLATION ############################
# Forked from naodevils-framework-base.sh

# install additional packages
mount -o bind /dev ./root/dev
mount -o bind /dev/pts ./root/dev/pts
mount -t sysfs /sys ./root/sys
mount -t proc /proc ./root/proc
chroot ./root /bin/bash <<"EOT"
set -e
set -o pipefail
DEBIAN_FRONTEND=noninteractive apt-get install -y alsa chrony gdb rsync espeak ntfs-3g exfat-fuse
exit
EOT
umount ./root/dev/pts
umount ./root/dev
umount ./root/sys
umount ./root/proc

# usb mounting service for boot
cat - <<"EOT" >> ./root/nao/.config/systemd/user/usb-mount.service
[Unit]
Description=Mount USB Drive at boot
After=local-fs.target
Before=runswift.service

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'for dev in $(lsblk -nlo NAME,TYPE | grep part | awk "{print \$1}"); do blkid /dev/$dev | grep -q <USB-UUID> && mount /dev/$dev /home/nao/logs; done'

[Install]
WantedBy=multi-user.target
EOT

# create systemd services
cat - <<"EOT" > ./root/nao/.config/systemd/user/runswift.service
[Unit]
Description=rUNSWift Framework
After=lola.service dev-video\x2dtop.device dev-video\x2dbottom.device usb-mount.service
Requires=lola.service dev-video\x2dtop.device dev-video\x2dbottom.device

StartLimitIntervalSec=120
StartLimitBurst=5

[Service]
Type=simple
LimitRTPRIO=36
ExecStart=sudo su nao -c /home/nao/bin/runswift
TimeoutStartSec=30
TimeoutStopSec=30
Restart=always
RestartSec=10s
StandardOutput=file:/home/nao/logs/runswift.stdout.log
StandardError=file:/home/nao/logs/runswift.stderr.log

[Install]
WantedBy=default.target
EOT

# stop runswift service from starting at boot
# but keep it so that it can be started manually for webnao
# ln -s ../runswift.service ./root/nao/.config/systemd/user/default.target.wants/runswift.service

# enable password root login
sed -i 's!#PermitRootLogin prohibit-password!PermitRootLogin yes!' ./root/etc/ssh/sshd_config

# add usb stick mount
cat - <<"EOT" >> ./root/etc/fstab
/dev/sda1            /home/nao/logs        auto       rw,noatime,noauto,user  0  0
EOT
mkdir ./root/nao/logs


# USB stick automation
# udev rules are running in a sandbox that doesn't allow to perform mount commands
# systemd services can call mount and unmount commands
# however systemd services can not be triggered from udev REMOVE rules
# that's why we have the following approach here:
# when usb stick is inserted, udev rule ADD calls a service
# usb-auto-mount@%k.service, where %k is the inserted device identifier, e.g. sda1
# this service mounts usb
# when usb stick is removed, udev rule REMOVE calls a script
# usb_unmount_s.sh %k, which in turn calls a service
# usb-auto-mount@%k.service , where %k is the removed device identifier, e.g. sda1
# this service unmounts usb

# usb mounting service
cat - <<"EOT" >> ./root/etc/systemd/system/usb-auto-mount@.service
[Unit]
Description=Mount USB Drive for %i

[Service]
Type=oneshot
ExecStart=mount /dev/%i /home/nao/logs
EOT

# usb unmounting service
cat - <<"EOT" >> ./root/etc/systemd/system/usb-auto-unmount@.service
[Unit]
Description=Unmount USB Drive for %i

[Service]
Type=oneshot
ExecStart=umount /home/nao/logs
EOT

# unmount usb script that calls unmount service
cat - <<"EOT" >> ./root/nao/usb_unmount_s.sh
#!/bin/sh
logfile=/tmp/usb.log
echo "-- `date`" >> $logfile
echo "usb_unmount_s $1" >> $logfile
sudo systemctl start usb-auto-unmount@$1.service
EOT

chmod +x ./root/nao/usb_unmount_s.sh

cat - <<"EOT" >> ./root/etc/udev/rules.d/99-usb-stick.rules
SUBSYSTEM=="block", KERNEL=="sd?[1-9]", ACTION=="add", RUN+="/bin/chgrp nao /dev/%k"
SUBSYSTEM=="block", KERNEL=="sd?[1-9]", ACTION=="add", ENV{SYSTEMD_WANTS}="usb-auto-mount@%k.service"
SUBSYSTEM=="block", KERNEL=="sd?[1-9]", ACTION=="remove", RUN+="/home/nao/usb_unmount_s.sh %k"
EOT

# set suid bit for ntfs and exfat support in user-space
chmod u+s ./root/usr/bin/ntfs-3g ./root/usr/sbin/mount.exfat-fuse

# add format usb script
cat - <<"EOT" >> ./root/usr/bin/format_usb
#!/bin/bash

set -e

# make sure usb is unmounted
umount /home/nao/logs || true

# wait some time
sleep 1

# partitioning
sfdisk --no-reread /dev/sda <<"EOF"
label: gpt
start=        2048, type=0FC63DAF-8483-4772-8E79-3D69D8477DE4
EOF

# wait for udev rules
sleep 1

mkfs.ext4 -E root_owner=1001:1001 /dev/sda1
EOT
chmod +x ./root/usr/bin/format_usb

# increase memory lock limit
sed -i 's!@rt              -       memlock         40000!@rt              -       memlock         3145728!' ./root/etc/security/limits.conf

# configure chrony
cat - <<"EOT" > ./root/etc/chrony/chrony.conf
server 10.1.0.1 iburst minpoll 0 maxpoll 6
# execute makestep in copyfiles
#makestep 1.0 3
driftfile /var/lib/chrony/chrony.drift
rtcfile /var/lib/chrony/chrony.rtc
dumponexit
dumpdir /var/lib/chrony
logdir /var/log/chrony
log statistics measurements tracking rtc
maxslewrate 0
EOT
sed -i 's#DAEMON_OPTS="-F -1"#DAEMON_OPTS="-r -s -F -1"#' ./root/etc/default/chrony

echo 'nao ALL=(ALL) NOPASSWD: /usr/bin/chronyc -n burst 2/10,/usr/bin/chronyc -n makestep 0.1 1' > ./root/etc/sudoers.d/chronyc
# allows user systemd to use sudo
sed -i 's/^%sudo\s*ALL=(ALL:ALL) ALL$/%sudo   ALL=(ALL:ALL) NOPASSWD:ALL/' ./root/etc/sudoers

# copy virtual audio device to mix input for alsa
cp "../../image/etc/asound.conf" ./root/etc/asound.conf

# allow ssh via the authorized_keys file
cp -r "../../image/home/nao/.ssh" ./root/nao/

# copy the runswift software if available
cp -r "../../image/home/nao/bin" ./root/nao/
cp "../../build-relwithdebinfo-native/robot/runswift" ./root/nao/bin/ || true
cp -r "../../image/home/nao/data" ./root/nao/
cp -r "../../robots/robots.cfg" ./root/nao/data/robots.cfg
cp -r "../../image/home/nao/whistle" ./root/nao/
cp -r "../../image/home/nao/.bashrc_ubuntu" ./root/nao/.bashrc

# give owner of the above back to nao
chown -R 1001:1001 ./root/nao

# add text-to-speech output of ethernet ip address
mkdir -p ./root/etc/networkd-dispatcher/configured.d
cat - <<"EOT" > ./root/etc/networkd-dispatcher/configured.d/say-ip
#!/bin/bash

set -e

if [ "$IFACE" = "eth0" ]; then
    read -r -a ip_addrs <<<"$IP_ADDRS"
    IP="I am $(hostname) and my IP is $(echo ${ip_addrs[0]} | sed 's/\./. /g')"
    espeak -a 100 -vf5 -p75 -g20 -m "$IP"
fi

exit 0
EOT

chmod +x ./root/etc/networkd-dispatcher/configured.d/say-ip

############################ REAL‐TIME PRIVILEGES SETUP ############################
chroot ./root /bin/bash <<"EOT"
grep -qxF 'nao    -    rtprio    99' /etc/security/limits.conf || \
  echo 'nao    -    rtprio    99' >> /etc/security/limits.conf
grep -qxF 'nao    -    memlock   unlimited' /etc/security/limits.conf || \
  echo 'nao    -    memlock   unlimited' >> /etc/security/limits.conf
EOT

if ! grep -q 'pam_limits.so' ./root/etc/pam.d/common-session; then
  sed -i '/^session required\s\+pam_unix.so/a session required pam_limits.so' \
    ./root/etc/pam.d/common-session
fi

chroot ./root /bin/bash <<"EOT"
set -e
# give non-root ability to set RT scheduling
setcap cap_sys_nice+ep /usr/bin/chrt

EOT

############################ END REAL‐TIME PRIVILEGES SETUP ############################

############################ END FRAMEWORK INSTALLATION ############################
