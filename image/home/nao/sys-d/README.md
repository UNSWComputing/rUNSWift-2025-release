# Post‑Flash Setup Guide for Runswift Robots

This short README walks you through the **first boot workflow** after you have flashed a robot with a fresh image.

---

## 1  Sync the code/serivce snappit from your PC to the robot

```bash
# On your development machine
make sync <robot_name>
```

---

## 2  Initialise systemd services on the robot

```bash
# On the robot (SSH session)
chmod +x /home/nao/bin/sys-d_init.sh
/home/nao/bin/sys-d_init.sh
```

The script:

1. Installs or updates the service unit files in `/etc/systemd/system`.
2. Performs a daemon‑reload so systemd picks up any changes.
3. Enables the Runswift services so they start automatically after every boot.

---

## 3  Reboot the robot

```bash
sudo reboot
```

---

## 4  Verify service status

Use `systemctl status` to make sure each component is running:

```bash
sudo systemctl status runswift_vision.service   # Vision pipeline
sudo systemctl status runswift_motion.service   # Motion control
sudo systemctl status runswift_stateestimation.service   # State estimation
```

> **Tip:** Substitute `vision`, `motion`, or any other component name to inspect a different service.

If a service fails, inspect its logs:

```bash
journalctl -u runswift_vision.service --no-pager
```


## KNOWN BUG
When the launch file is launched by sys-d, running ros2 topic list will return nothing. However, if running other nodes from the terminal such as
Keyboard teleop it still connect to all the nodes properly.