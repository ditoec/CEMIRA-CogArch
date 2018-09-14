#!/bin/bash
sudo chmod a+rw /dev/ttyACM0
sleep 9
roslaunch head head_interface.launch
