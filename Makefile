.PHONY: build run-pub run-sub launch-pub-sub
.DEFAULT_GOAL := build

build:
	cd ../../../../; \
		colcon build --packages-select demo

run-pub:
	cd ../../../../; \
		rosrun demo pub

run-sub:
	cd ../../../../; \
		rosrun demo sub

launch-pub-sub:
	cd ../../../../; \
		roslaunch demo pub_sub.launch