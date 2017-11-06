#!/usr/bin/env python

from arm.srv import *
import sys
import copy
import rospy
import moveit_commander
import yumi_moveit_utils as yumi
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty

def run():
    """Starts the node

    Runs to start the node and initialize everthing. Runs forever via Spin()

    :returns: Nothing
    :rtype: None
    """
    rospy.init_node('arm_server')
    
    #Start by connecting to ROS and MoveIt!
    yumi.init_Moveit()

    #yumi.move_point(yumi.BOTH, 0.35, -0.25)
    #yumi.move_pick(yumi.BOTH, 0.35, -0.25)
    #yumi.move_handover_ready(yumi.BOTH,0.55,-0.20)
    #yumi.move_handover_give(yumi.BOTH,0.55,-0.20)

    rospy.Service('arm/move_home', ArmInterface, handle_move_home)
    rospy.Service('arm/open_gripper', ArmInterface, handle_open_gripper)
    rospy.Service('arm/close_gripper', ArmInterface, handle_close_gripper)
    rospy.Service('arm/move_simple', ArmInterface, handle_move_simple)
    rospy.Service('arm/move_point', ArmInterface, handle_move_point)
    rospy.Service('arm/move_pick', ArmInterface, handle_move_pick)
    rospy.Service('arm/move_place', ArmInterface, handle_move_place)
    rospy.Service('arm/move_handover_ready', ArmInterface, handle_move_handover_ready)
    rospy.Service('arm/move_handover_give', ArmInterface, handle_move_handover_give)
    rospy.Service('arm/move_handover_take', ArmInterface, handle_move_handover_take)

    print "Arm Server is running.. "
    rospy.on_shutdown(yumi.kill_Moveit)
    rospy.spin()

def handle_move_home(req):
    print "arm - move_home - request received! Executing..."
    result = yumi.move_home(req.arm)
    if (result==1):
	"arm - move_home - request executed !"
    else:
	"arm - move_home - execution failed !"
    return ArmInterfaceResponse(result)

def handle_open_gripper(req):
    print "arm - open_gripper - request received! Executing..."
    result = yumi.open_grippers(req.arm)
    if (result==1):
	"arm - open_gripper - request executed !"
    else:
	"arm - open_gripper - execution failed !"
    return ArmInterfaceResponse(result)

def handle_close_gripper(req):
    print "arm - close_gripper - request received! Executing..."
    result = yumi.close_grippers(req.arm)
    if (result==1):
	"arm - close_gripper - request executed !"
    else:
	"arm - close_gripper - execution failed !"
    return ArmInterfaceResponse(result)

def handle_move_simple(req):
    print "arm - move_simple - request received! Executing..."
    result = yumi.move_simple(req.arm, create_pose_euler(req.x, req.y, req.z, req.roll, req.pitch, req.yaw))
    if (result==1):
	"arm - move_simple - request executed !"
    else:
	"arm - move_simple - execution failed !"
    return ArmInterfaceResponse(result)

def handle_move_point(req):
    print "arm - move_point - request received! Executing..."
    result = yumi.move_point(req.arm, req.x, req.y)
    if (result==1):
	"arm - move_point - request executed !"
    else:
	"arm - move_point - execution failed !"
    return ArmInterfaceResponse(result)
    
def handle_move_pick(req):
    print "arm - move_pick - request received! Executing..."
    result = yumi.move_pick(req.arm, req.x, req.y)
    if (result==1):
	"arm - move_pick - request executed !"
    else:
	"arm - move_pick - execution failed !"
    return ArmInterfaceResponse(result)

def handle_move_place(req):
    print "arm - move_place - request received! Executing..."
    result = yumi.move_place(req.arm, req.x, req.y)
    if (result==1):
	"arm - move_place - request executed !"
    else:
	"arm - move_place - execution failed !"
    return ArmInterfaceResponse(result)

def handle_move_handover_ready(req):
    print "arm - move_handover_ready - request received! Executing..."
    result = yumi.move_handover_ready(req.arm, req.x, req.y, req.z)
    if (result==1):
	"arm - move_handover_ready - request executed !"
    else:
	"arm - move_handover_ready - execution failed !"
    return ArmInterfaceResponse(result)
    
def handle_move_handover_give(req):
    print "arm - move_handover_give - request received! Executing..."
    result = yumi.move_handover_give(req.arm, req.x, req.y, req.z)
    if (result==1):
	"arm - move_handover_give - request executed !"
    else:
	"arm - move_handover_give - execution failed !"
    return ArmInterfaceResponse(result)

def handle_move_handover_take(req):
    print "arm - move_handover_take - request received! Executing..."
    result = yumi.move_handover_take(req.arm, req.x, req.y, req.z)
    if (result==1):
	"arm - move_handover_take - request executed !"
    else:
	"arm - move_handover_take - execution failed !"
    return ArmInterfaceResponse(result)

if __name__ == '__main__':
    try:
        run()
    	print "####################################     Arm Server Terminated     ####################################"
    except rospy.ROSInterruptException:
        pass
