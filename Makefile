# This Makefile documents common commands you'd run in this repo

# Define ARGS to capture extra arguments
ARGS = $(filter-out $@,$(MAKECMDGOALS))
TARGET := $(word 1,$(ARGS))

$(eval $(wordlist 2,$(words $(ARGS)),$(ARGS)):;@:)

SHELL = /usr/bin/env bash
.PHONY: build

help:
	@echo "Environment Setup"
	@echo -e " * dev \t\t\t\tEnter the rUNSWift devcontainer, or Ctrl+Shift+P in VSCode, Reopen In Container."
	@echo -e " * undev \t\t\tTear down the devcontainer. e.g. use when you want to free up space."
	@echo -e " * setup \t\t\tInstalls proprietary build dependencies."
	@echo -e "\t\t\t\tIntended to be run in the devcontainer, please run if repo is freshly cloned."

	@echo -e "\nImage Building"
	@echo -e " * build-image \t\t\tBuild the rUNSWift OS image meant to run on the robot. Outputs an OPN file."
	@echo -e " * secrets \t\t\tClone team secrets into working tree. Note: You may need to request access."

	@echo -e "\nFrequently Used"
	@echo -e " * sync <robot> [pkg1 pkg2...]\tSync repo to the robot. Warning: Can overwrite robot's bashrc."

	@if [ ! -d /workspace ]; then true; echo -e "\nWARNING: You are not inside the devcontainer! There is a 99% chance you will run into issues.\n   HINT: Run \`make dev\`, or use VSCode Devcontainers."; fi

### Building images and binaries
setup:
	cd ./robot_ws && ./src/3rdparty/scripts/install-dependencies.sh

# when building the image, it's recommended you have rUNSWift already built so it can be included in the image
build-image:
	./bin/build-naoimage.sh

# pull secrets repository into secrets/ folder for use in scripts
secrets:
	git clone --depth=1 git@github.com:unswcomputing/runswift-secrets ./secrets && rm -rf ./secrets/.git

### While at Comp
# Changes the wifi the robot connects to
change-wifi: # usage: change-wifi <robot/all> <wifi-name/NONE>
	./bin/nao-util.py $(firstword $(ARGS)) --wifi $(word 2,$(ARGS))

build:
	cd robot_ws && colcon build --cmake-args ' -DCMAKE_BUILD_TYPE=Release'

sync:
ifeq ($(word 2,$(ARGS)),)
	@echo -e "usage: sync <robot> [package1 package2...]\t\tSync repo to the robot. Warning: Can overwrite robot's bashrc."
else ifeq ($(word 3,$(ARGS)),)
	./bin/nao-util.py $(ARGS) --sync
else
	./bin/nao-util.py $(firstword $(ARGS)) --sync-pkg $(shell echo "$(wordlist 2,$(words $(ARGS)),$(ARGS))" | tr ' ' ',')
endif

# shutdown a nao robot (or all of them)
nao-shutdown: # usage: make nao-shutdown <robot/all>
	./bin/nao-util.py $(ARGS) --shutdown

# reboot a nao robot (or all of them)
nao-reboot: # usage: make nao-reboot <robot/all>
	./bin/nao-util.py $(ARGS) --reboot

# reboot a nao robot (or all of them)
nao-check: # usage: make nao-reboot <robot/all>
	./bin/nao-util.py $(ARGS) --check

### Misc
# adds the robots to your hosts file, meaning you can easily reference them in any command like ping/ssh without needing its ip
update-hosts:
	sudo ./bin/update-hosts.sh

clean:
	rm -rf build-*
	rm -rf robot_ws/{build,install,log}

### ROS2
# Get inside of a dev environment
ros-run-docker:
	docker compose up -d --no-recreate
	docker compose exec -it dev bash

ros-build-docker:
	docker compose build --pull

ros-pull-docker:
	docker compose pull

ros-stop-docker:
	docker compose kill

ros-rm-docker:
	docker compose rm

dev: ros-run-docker

undev:
	docker compose down -t 1

# Converts old bbd2 dumps to ROS2 bags. Usage: `make ros-dump-to-bag -- --help`. E.g `make ros-dump-to-bag -- file.bbd2`.
# this app opens a display to show the frame its up to. You can run headless by using the option -d (if you don't wish to open a display in the docker container)
ros-dump-to-bag:
	./bin/ros-docker.sh "cd ./utils/ros2-dump-to-bag && cargo run --release -- $(ARGS)"

### Fun
# Get a robot to say something
nao-say: # usage: make nao-say <robot/all> "<string>"
	./bin/nao-util.py $(firstword $(ARGS)) --say "$(wordlist 2,$(words $(ARGS)),$(ARGS))"

