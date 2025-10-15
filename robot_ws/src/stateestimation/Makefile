SHELL := /usr/bin/bash

.PHONY: build
build:
	cd ../.. && \
	colcon build --packages-up-to stateestimation \
		         --cmake-args ' -DCMAKE_EXPORT_COMPILE_COMMANDS=ON' ' -DCMAKE_BUILD_TYPE=RelWithDebInfo'

builddebug:
	cd ../.. && \
	colcon build --packages-up-to stateestimation \
		         --cmake-args ' -DCMAKE_EXPORT_COMPILE_COMMANDS=ON' ' -DCMAKE_BUILD_TYPE=Debug'

runself:
	@cd ../.. && source ./install/setup.sh && \
	ros2 run stateestimation SelfEstimator

debugself:
	@cd ../.. && source ./install/setup.sh && \
	ros2 run --prefix 'gdbserver localhost:3000' stateestimation SelfEstimator

runball:
	@cd ../.. && source ./install/setup.sh && \
	ros2 run stateestimation BallEstimator

debugball:
	@cd ../.. && source ./install/setup.sh && \
	ros2 run --prefix 'gdbserver localhost:3000' stateestimation BallEstimator

