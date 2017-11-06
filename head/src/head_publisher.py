#!/usr/bin/env python

import rospy
import time
import math
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.msg import MotorStateList

head_pan = 0.0
head_tilt = 0.0

def head_publisher():
	global head_pan
	global head_tilt
	head_pub = rospy.Publisher('joint_states', JointState , queue_size=10)
        rospy.Subscriber("/position_control/dynamixel_states", MotorStateList, head_joint_callback)
	rospy.init_node('head_publisher', anonymous=False)
	rate = rospy.Rate(10)
    	while not rospy.is_shutdown():
		head_joint_state = JointState()
		head_joint_state.header.stamp = rospy.get_rostime()
		head_joint_state.name.append("head_pan_joint")
		head_joint_state.position.append(head_pan)
		head_joint_state.name.append("head_tilt_joint")
		head_joint_state.position.append(head_tilt)
		head_pub.publish(head_joint_state)
		rate.sleep()

def head_joint_callback():
	global head_pan
	global head_tilt
	head_pan = ((data[0].present_position - 512)/1024)*(5/6)*math.PI
	head_tilt = ((data[1].present_position - 512)/1024)*(5/6)*math.PI


if __name__ == '__main__':
	try:
		head_publisher()
	except rospy.ROSInterruptException:
		pass
