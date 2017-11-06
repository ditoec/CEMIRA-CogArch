#!/usr/bin/env python

from head.srv import *
from dynamixel_workbench_msgs.srv import SetPosition
import sys
import copy
import rospy
import math

def run():
    """Starts the node

    Runs to start the node and initialize everthing. Runs forever via Spin()

    :returns: Nothing
    :rtype: None
    """
    rospy.init_node('head_server')
    
    rospy.Service('head/move_head', HeadInterface, handle_move_head)

    print "Head Server is running.. "
    rospy.spin()

def handle_move_head(req):
    print "move_head - request received! Executing..."
    rospy.wait_for_service('/joint_command')
    headControl = rospy.ServiceProxy('/joint_command', SetPosition)
    pan = math.atan2(req.y,(req.x+0.1)) #use a static X offset here
    if (pan < - (math.PI/4)):
	 pan = -math.PI/4
    elif (pan > (math.PI/4)): 
	pan = math.PI/4
    tilt = math.atan2((req.z-0.6),(req.x+0.1)) #use a static Z offset here
    if (tilt < - (math.PI/8)):
	tilt = -math.PI/8
    if (tilt > (math.PI/8)):
	tilt = math.PI/8

    resp = headControl('rad',pan,tilt)
    if (resp.pan_pos==pan and resp.tilt_pos==tilt):
	result = 1
	"move_head - request executed !"
    else:
	result = 0
	"move_head - execution failed !"
    return HeadInterfaceResponse(result)

if __name__ == '__main__':
    try:
        run()
    	print "####################################     Head Server Terminated     ####################################"
    except rospy.ROSInterruptException:
        pass
