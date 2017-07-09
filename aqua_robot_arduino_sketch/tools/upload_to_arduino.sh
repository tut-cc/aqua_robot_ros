#!/bin/bash

RASPBERRY_IP=192.168.1.2
RASPBERRY_USER=tut

scp -r ~/catkin_ws/src/aqua_robot_ros/aqua_robot_messages ${RASPBERRY_USER}@${RASPBERRY_IP}:/home/tut/catkin_ws/src/
scp -r ~/catkin_ws/src/aqua_robot_ros/aqua_robot_arduino_sketch/AquaRobotArduino ${RASPBERRY_USER}@${RASPBERRY_IP}:/home/tut/Arduino/
ssh ${RASPBERRY_USER}@${RASPBERRY_IP} 'bash ~/update_arduino_libraries_and_upload_sketch.sh'
