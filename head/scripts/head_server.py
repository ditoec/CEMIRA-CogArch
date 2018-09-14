#!/usr/bin/env python

from head.srv import *
from dynamixel_workbench_msgs.srv import JointCommand
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
    headControl = rospy.ServiceProxy('/joint_command', JointCommand)
    pan = math.atan2(-req.y,(req.x)) #use a static X offset here
    if (pan < - (math.pi/4)):
	 pan = -math.pi/4
    elif (pan > (math.pi/4)): 
	pan = math.pi/4
    tilt = math.atan2((req.z),(req.x)) #use a static Z offset here
    if (tilt < - 0.25):
	tilt = -0.25
    if (tilt > 0):
	tilt = 0

    resp1 = headControl('rad',2,pan)
    resp2 = headControl('rad',1,tilt)
    if (resp1.result==1 and resp2.result==1):
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
