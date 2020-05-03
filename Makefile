
.PHONY: build run-pub run-sub
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
