cp /home/nao/sys-d/runswift_motion.service /etc/systemd/system/runswift_motion.service
cp /home/nao/sys-d/runswift_vision.service /etc/systemd/system/runswift_vision.service
cp /home/nao/sys-d/runswift_stateestimation.service /etc/systemd/system/runswift_stateestimation.service
cp /home/nao/sys-d/runswift_comms.service /etc/systemd/system/runswift_comms.service
cp /home/nao/sys-d/runswift_behaviours.service /etc/systemd/system/runswift_behaviours.service
cp /home/nao/sys-d/runswift_hri.service /etc/systemd/system/runswift_hri.service
cp /home/nao/sys-d/runswift_startup.service /etc/systemd/system/runswift_startup.service
cp /home/nao/sys-d/runswift_errors.service /etc/systemd/system/runswift_errors.service
cp /home/nao/sys-d/foxglove.service /etc/systemd/system/foxglove.service

sudo chmod 644 /etc/systemd/system/runswift_motion.service
sudo chmod 644 /etc/systemd/system/runswift_vision.service
sudo chmod 644 /etc/systemd/system/runswift_stateestimation.service
sudo chmod 644 /etc/systemd/system/runswift_comms.service
sudo chmod 644 /etc/systemd/system/runswift_behaviours.service
sudo chmod 644 /etc/systemd/system/runswift_hri.service
sudo chmod 644 /etc/systemd/system/runswift_startup.service
sudo chmod 644 /etc/systemd/system/runswift_errors.service
sudo chmod 644 /etc/systemd/system/foxglove.service

sudo systemctl daemon-reload
sudo systemctl enable runswift_vision.service
sudo systemctl enable runswift_stateestimation.service
sudo systemctl enable runswift_comms.service
sudo systemctl enable runswift_motion.service
sudo systemctl enable runswift_behaviours.service
sudo systemctl enable runswift_hri.service
sudo systemctl enable runswift_startup.service
sudo systemctl enable runswift_errors.service
