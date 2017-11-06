#!/bin/bash
export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x64
rosrun leap_motion sender.py &
