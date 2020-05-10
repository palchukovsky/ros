.PHONY: build run-pub run-pub-py run-sub run-sub-py launch-pub-sub launch-pub-sub-py
.DEFAULT_GOAL := build

build:
	cd ../../../../; \
		colcon build --packages-select demo

run-pub:
	cd ../../../../; \
		rosrun demo pub

run-pub-py:
	cd ../../../../; \
		rosrun demo scripts/pub.py

run-sub:
	cd ../../../../; \
		rosrun demo sub

run-sub-py:
	cd ../../../../; \
		rosrun demo scripts/sub.py

launch-pub-sub:
	cd ../../../../; \
		roslaunch demo pub_sub.launch

launch-pub-sub-py:
	cd ../../../../; \
		roslaunch demo pub_sub_py.launch