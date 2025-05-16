#!/bin/bash

source ~/esp32bot_ws/install/setup.bash 
ros2 launch gesture_recognizer gesture_recognizer.launch.py &
ros2 launch esp32bot_controller joy_teleop.launch.py &
ros2 launch esp32bot_detect esp32bot_detect.launch.py &
rqt

wait