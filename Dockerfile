FROM ubuntu:22.04

# set timezone
ENV TZ=UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# ---- install build essential tools
RUN apt update && apt install -y build-essential locales software-properties-common sudo python3 python3-pip curl git

# ---- install ros2 tools:
# add apt repo for ros2 humble (source: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#resources)
RUN add-apt-repository universe

# add ros2 repo gpg key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# add ros2 repo to sources list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# update packages from ros2 repository
RUN apt update && apt upgrade -y

# install ros2 packages we use
RUN apt install -y ros-humble-desktop-full python3-rosdep ros-dev-tools
# install opencv
RUN pip3 install --upgrade numpy==1.26.4
RUN sudo apt-get install -y libopencv-dev
###### Try avoid modifying anything above this line! The above commands take a very long time to run, we'd like to use the cache when possible
RUN apt update && apt install -y ros-humble-nao-lola ros-humble-usb-cam ros-humble-vision-msgs ros-humble-tf-transformations ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-joint-state-publisher-gui

# --------------------------------------------------
ARG USERNAME=ubuntu

# create nonroot user
RUN useradd -m -G uucp,video -s /bin/bash $USERNAME

# sudo with no password
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME

# bashrc config
RUN echo 'PATH=/workspace/bin:/workspace/bin/naoctl:$PATH' >> ~/.bashrc

# configure locales
RUN sudo locale-gen en_US en_US.UTF-8 && \
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# add locales to bashrc config`
RUN echo 'export LANG=en_US.UTF-8' >> ~/.bashrc

# rosdep update
RUN sudo rosdep init && rosdep update

# add new ros setup to bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo "export ws_path=/workspace/robot_ws" >> ~/.bashrc

# disable .known_hosts completely for ssh
# this allows easy ssh into robots and other machines
# without having to worry about the known_hosts file
# RUN mkdir -p ~/.ssh && \
# echo "StrictHostKeyChecking no\n" \
# "UserKnownHostsFile /dev/null\n" \
# "LogLevel ERROR\n" \
# >> ~/.ssh/config

# install tools required for naoimage to run
RUN sudo apt install -y vim nano tmux htop tree screen iproute2 traceroute iputils-ping sshpass \
                        rsync debootstrap pigz e2fsprogs patchelf xxd bsdmainutils ninja-build

# to enable vnc (remote desktop)
RUN sudo apt install -y novnc x11vnc xvfb

RUN echo '#!/bin/bash\n\
sudo chmod 1777 /tmp/.X11-unix\n\
Xvfb :1 -screen 0 1920x1080x24 & \n\
export DISPLAY=:1 \n\
x11vnc -display :1 -nopw -forever & \n\
/usr/share/novnc/utils/launch.sh --vnc localhost:5900 --listen 8080 & \n\
' > ~/entryfile.sh \
    && chmod +x ~/entryfile.sh

# install rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH=/home/ubuntu/.cargo/bin:$PATH
RUN pip install git+https://github.com/colcon/colcon-cargo.git
RUN pip install git+https://github.com/colcon/colcon-ros-cargo.git
RUN cargo install cargo-ament-build

# needed for building ros2 rust packages (and other rust tools we use)
RUN sudo apt install -y libclang-dev libsqlite3-dev

# pygame,onnxruntime, transforms3d needed for behvaiour sim render (bviz)
# construct needed for running comms locally
RUN pip3 install pygame onnxruntime==1.20.1 transforms3d construct

# auto-source ros2
RUN echo "source /workspace/robot_ws/install/setup.bash" >> ~/.bashrc


RUN sudo mkdir -p /etc/apt/keyrings \
&& cd /etc/apt/keyrings \
&& sudo wget -q https://cyberbotics.com/Cyberbotics.asc \
&& echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list \
&& sudo apt update

RUN echo "keyboard-configuration keyboard-configuration/layout select English (Australian)" | sudo debconf-set-selections \
&& echo "keyboard-configuration keyboard-configuration/variant select English (Australian)" | sudo debconf-set-selections \
&& echo "keyboard-configuration keyboard-configuration/model select Generic 105-key PC" | sudo debconf-set-selections \
&& echo "keyboard-configuration keyboard-configuration/layoutcode select au" | sudo debconf-set-selections \
&& echo "keyboard-configuration keyboard-configuration/altgr select The default for the keyboard layout" | sudo debconf-set-selections \
&& echo "keyboard-configuration keyboard-configuration/compose select No compose key" | sudo debconf-set-selections \
&& echo "keyboard-configuration keyboard-configuration/ctrl_alt_bksp boolean false" | sudo debconf-set-selections
RUN sudo apt -y install --no-install-recommends webots || echo "webots installation failed -- You are likely build on ARM64 architecture"

# nao-sync dependency
RUN sudo pip install paramiko

# allow running makefile from any subfolder, and ensure colcon is always running in robot_ws
RUN echo 'make () {\n\
    if [[ ! -f ./Makefile ]]; then\n\
        return_folder=$(pwd)\n\
        cd /workspace\n\
    fi\n\
    \n\
    /usr/bin/make $@\n\
    \n\
    if [[ ! -z $return_folder ]]; then\n\
        cd $return_folder\n\
    fi\n\
}\n\
\n\
colcon () {\n\
    if [[ "$(pwd)" != "/workspace/robot_ws" ]]; then\n\
        echo "note: entering robot_ws before running colcon, use /usr/bin/colcon to bypass"\n\
        return_folder=$(pwd)\n\
        cd /workspace/robot_ws\n\
    fi\n\
\n\
    /usr/bin/colcon $@\n\
\n\
    if [[ ! -z $return_folder ]]; then\n\
        cd $return_folder\n\
    fi\n\
}\n\
' >> ~/.bashrc

# colcon autocompletion
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# fgb and madgwick filter
RUN sudo apt install -y ros-humble-foxglove-bridge ros-humble-imu-tools