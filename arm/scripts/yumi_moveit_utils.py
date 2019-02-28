#!/usr/bin/env python

import sys
import copy
import rospy
import tf
import moveit_commander
from moveit_commander import *
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
from yumi_hw.srv import *

LEFT = 2        #:ID of the left arm
RIGHT = 1       #:ID of the right arm
BOTH = 3        #:ID of both_arms
PI = 3.1415926535  #:Value of PI
HALF_PI = PI/2

table_height = 0#:The height of the upper surface of the table

global group_l  #:The move group for the left arm
global group_r  #:The move group for the right arm
global group_both #:The move group for using both arms at once
global robot    #:The RobotCommander() from MoveIt!
global scene    #:The PlanningSceneInterface from MoveIt!
global pose_r
global pose_l
global gripper_r
global gripper_l
global last_arm
global plan_l
global plan_r
global plan_both

#------------------------LOW LEVEL FUNCTIONS-------------------------------------------#


#Initializes the package to interface with MoveIt!
def init_Moveit():
    """Initializes the connection to MoveIt!

    Initializes all the objects related to MoveIt! functions. Also adds in the
    table to the MoveIt! planning scene.

    :returns: Nothing
    :rtype: None
    """

    global group_l
    global group_r
    global group_both
    global robot
    global scene
    global pose_r
    global pose_l
    print("####################################     Start Initialization     ####################################")
    moveit_commander.roscpp_initialize(sys.argv)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    #clean the scene
    #scene.remove_world_object("table")

    # publish a demo scene
    #p = PoseStamped()
    #p.header.frame_id = "yumi_body"

    # add a table
    #p.pose.position.x = 0.325
    #p.pose.position.y = 0.0
    #p.pose.position.z = table_height
    #scene.add_box("table", p, (0.65, 0.65, 0))
    
    group_l = moveit_commander.MoveGroupCommander("left_arm")
    group_l.set_planner_id("RRTstarkConfigDefault")
    group_l.set_pose_reference_frame("yumi_body")
    group_l.allow_replanning(True)
    group_l.set_goal_position_tolerance(0.001)
    group_l.set_goal_orientation_tolerance(0.005)

    group_r = moveit_commander.MoveGroupCommander("right_arm")
    group_r.set_planner_id("RRTstarkConfigDefault")
    group_r.set_pose_reference_frame("yumi_body")
    group_r.allow_replanning(True)
    group_r.set_goal_position_tolerance(0.001)
    group_r.set_goal_orientation_tolerance(0.005)

    group_both = moveit_commander.MoveGroupCommander("both_arms")
    group_both.set_planner_id("RRTstarkConfigDefault")
    group_both.set_pose_reference_frame("yumi_body")
    group_both.allow_replanning(True)
    group_both.set_goal_position_tolerance(0.001)
    group_both.set_goal_orientation_tolerance(0.005)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 	moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    rospy.Subscriber("/yumi/gripper_states", JointState, gripper_callback)
    rospy.sleep(1)
    print("####################################     Finished Initialization     ####################################")

    sys.stdout.write('\nYuMi MoveIt! util initialized!\n\n\n')
    rospy.loginfo('Reset Arm Position to Home!')
    move_home(BOTH)
    pose_r = group_r.get_current_pose().pose
    pose_l = group_l.get_current_pose().pose

def kill_Moveit():
	moveit_commander.roscpp_shutdown()
	print "============ STOPPING Moveit!==============="
    
def gripper_callback(data):
    global gripper_r
    global gripper_l
    gripper_l = data.position[0]
    gripper_r = data.position[1]

def print_current_pose(arm):
    """Prints the current pose

    Prints the current pose of the selected arm into the terminal

    :param arm: The arm to display the pose of (RIGHT or LEFT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    pose = get_current_pose(arm)
    rpy = get_current_rpy(arm)

    if (arm == LEFT):
        print("Left arm pose and rpy:")
    if (arm == RIGHT):
        print("Right arm pose and rpy:")

    print(pose)
    print(rpy)

def get_current_pose(arm):
    """Gets the current pose

    Return the current pose of the selected arm

    :param arm: The arm to display the pose of (RIGHT or LEFT)
    :type arm: int
    :returns: Pose
    :rtype: PoseStamped
    """
    global group_l
    global group_r
    if (arm == LEFT):
        cur_arm = group_l
    if (arm == RIGHT):
        cur_arm = group_r
    return cur_arm.get_current_pose()

def get_current_rpy(arm):
    """Gets the current orientation

    Returns the current orientation of the selected arm as Euler angles

    :param arm: The arm to display the pose of (RIGHT or LEFT)
    :type arm: int
    :returns: Orientation
    :rtype: Orientation
    """
    global group_l
    global group_r
    if (arm == LEFT):
        cur_arm = group_l
    if (arm == RIGHT):
        cur_arm = group_r
    return cur_arm.get_current_rpy()

def get_current_joint_values(sel):
    """Gets the current joint values

    Returns the current position of all joints in the selected arm. From J1-J7

    :param sel: The arm to display the pose of (RIGHT or LEFT)
    :type sel: int
    :returns: Joint values
    :rtype: float[]
    """
    global group_l
    global group_r

    if (sel == RIGHT):
        cur_arm = group_r
    if (sel == LEFT):
        cur_arm = group_l

    return cur_arm.get_current_joint_values()

def print_current_joint_states(arm):
    """Prints the current joint values

    Prints the current joint values of the selected arm into the terminal

    :param arm: The arm to display the pose of (RIGHT or LEFT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """

    if (arm == LEFT):
        print("Left arm joint states:")
    if (arm == RIGHT):
        print("Right arm joint states:")

    print(get_current_joint_values(arm))

#Create a pose object from position and orientation (euler angles)
def create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad):
    """Creates a pose using euler angles

    Creates a pose for use with MoveIt! using XYZ coordinates and RPY
    orientation in radians

    :param x_p: The X-coordinate for the pose
    :param y_p: The Y-coordinate for the pose
    :param z_p: The Z-coordinate for the pose
    :param roll_rad: The roll angle for the pose
    :param pitch_rad: The pitch angle for the pose
    :param yaw_rad: The yaw angle for the pose
    :type x_p: float
    :type y_p: float
    :type z_p: float
    :type roll_rad: float
    :type pitch_rad: float
    :type yaw_rad: float
    :returns: Pose
    :rtype: PoseStamped
    """
    quaternion = tf.transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
    return create_pose(x_p, y_p, z_p, quaternion[0], quaternion[1], quaternion[2], quaternion[3])

#Create a pose object from position and orientation (quaternion)
def create_pose(x_p, y_p, z_p, x_o, y_o, z_o, w_o):
    """Creates a pose using quaternions

    Creates a pose for use with MoveIt! using XYZ coordinates and XYZW
    quaternion values

    :param x_p: The X-coordinate for the pose
    :param y_p: The Y-coordinate for the pose
    :param z_p: The Z-coordinate for the pose
    :param x_o: The X-value for the orientation
    :param y_o: The Y-value for the orientation
    :param z_o: The Z-value for the orientation
    :param w_o: The W-value for the orientation
    :type x_p: float
    :type y_p: float
    :type z_p: float
    :type x_o: float
    :type y_o: float
    :type z_o: float
    :type w_o: float
    :returns: Pose
    :rtype: PoseStamped
    """
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x_p
    pose_target.position.y = y_p
    pose_target.position.z = z_p
    pose_target.orientation.x = x_o
    pose_target.orientation.y = y_o
    pose_target.orientation.z = z_o
    pose_target.orientation.w = w_o
    return pose_target

#Set the gripper to an effort value
def gripper_effort(gripper_id, effort):
    """Set gripper effort

    Sends an effort command to the selected gripper. Should be in the range of
    -20.0 (fully open) to 20.0 (fully closed)

    :param gripper_id: The ID of the selected gripper (LEFT or RIGHT)
    :param effort: The effort value for the gripper (-20.0 to 20.0)
    :type gripper_id: int
    :type effort: float
    :returns: Nothing
    :rtype: None
    """
    rospy.loginfo('Setting gripper effort to ' + str(effort) + ' for arm ' + str(gripper_id))
    
    if (gripper_id == RIGHT):
        pubname = '/yumi/gripper_r_effort_cmd'

    if (gripper_id == LEFT):
        pubname = '/yumi/gripper_l_effort_cmd'

    pub = rospy.Publisher(pubname, std_msgs.msg.Float64, queue_size=10, latch=True)
    pub.publish(std_msgs.msg.Float64(effort))

#Plan a cartesian path
def plan_path(arm, points, planning_tries = 500):
    global robot
    global group_l
    global group_r

    if (arm == LEFT):
        armname = 'left'
        cur_arm = group_l
    if (arm == RIGHT):
        armname = 'right'
        cur_arm = group_r

    rospy.loginfo('Moving ' + armname + ' through path: ')
    rospy.loginfo(points)

    waypoints = []
    #waypoints.append(cur_arm.get_current_pose().pose)
    for point in points:
        wpose = create_pose_euler(point[0], point[1], point[2], point[3], point[4], point[5])
        waypoints.append(copy.deepcopy(wpose))
    #print waypoints

    cur_arm.set_start_state_to_current_state()

    attempts = 0
    fraction = 0.0

    while fraction < 1.0 and attempts < planning_tries:
        (plan, fraction) = cur_arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)
        attempts += 1
        rospy.loginfo('attempts: ' + str(attempts) + ', fraction: ' + str(fraction))
        if (fraction == 1.0):
            plan = cur_arm.retime_trajectory(robot.get_current_state(), plan, 1.0)
            return plan
            #r = cur_arm.execute(plan)

    if (fraction < 1.0):
        rospy.logerr('Only managed to calculate ' + str(fraction*100) + '% of the path!')
        raise Exception('Could not calculate full path, exiting')

    return None

#Plan a cartesian path through the given waypoints and execute
def traverse_path(arm, points, planning_tries = 500):
    """Commands an end-effector to traverse a path

    Creates a path between the given waypoints, interpolating points spaced
    by 1cm. Then tries to plan a trajectory through those points, until it
    succeeds and executes or if it fails to find a path for 500 tries it throws
    an exception.

    :param points: An array of waypoints which are themselves array of the form [x,y,z,r,p,y]
    :param arm: The selected arm (LEFT or RIGHT)
    :param planning_tries: The number of tries allowed for planning (default=100)
    :type points: float[[]]
    :type arm: int
    :type planning_tries: int
    :returns: Nothing
    :rtype: None
    """
    global group_l
    global group_r

    if (arm != BOTH):
        plan = plan_path(arm, points, planning_tries)
        if (plan != None):
            if(arm == RIGHT):
                group_r.execute(plan)
            else:
                group_l.execute(plan)
    else:
        traverse_pathDual(points, planning_tries)

#Plan a trajectory
def plan(move_group, target):
    """Plans a group to target

    Creates a plan to move a move_group to the given target.

    :param move_group: The group to move
    :param target: The pose to move to
    :type move_group: MoveGroup
    :type target: PoseStamped
    :returns: Nothing
    :rtype: None
    """
    global group_l
    global group_r
    global group_both
    global plan_l
    global plan_r
    global plan_both
    success = 0
    if (move_group == group_both):
	rospy.loginfo('Planning both arm!')
	move_group.set_pose_target(target[0],group_l.get_end_effector_link())
	move_group.set_pose_target(target[1],group_r.get_end_effector_link())
        plan_both = None
	plan_both = group_both.plan()
	if (plan_both.joint_trajectory.points > 0):
	    rospy.loginfo('planning ' + 'both' + ' arm success !')
	    success = 1
	else:
	    rospy.loginfo('planning ' + 'both' + ' arm failed !')
    else:
	rospy.loginfo('Planning arm: Position: {' + str(target.position.x) + ';' + str(target.position.y) + ';' + str(target.position.z) + '}')
	move_group.set_pose_target(target)
        if (move_group == group_l):
            arm = 'left'
	    plan_l = None
	    plan_l = group_l.plan()
	    if (plan_l.joint_trajectory.points > 0):
	        rospy.loginfo('planning ' + arm + ' arm success !')
		success = 1
	    else:
	        rospy.loginfo('planning ' + arm + ' arm failed !')
	elif (move_group == group_r):
            arm = 'right'
	    plan_r = None
	    plan_r = group_r.plan()
	    if (plan_r.joint_trajectory.points > 0):
	        rospy.loginfo('planning ' + arm + ' arm success !')
		success = 1
	    else:
	        rospy.loginfo('planning ' + arm + ' arm failed !')
    return success
	
#Execute a trajectory
def move(move_group,should_wait=True):
    global group_l
    global group_r
    global group_both
    global plan_l
    global plan_r
    global plan_both
    success = 0
    if (move_group == group_l):
        arm = 'left'
	if(plan_l.joint_trajectory.points > 0):
	    success = group_l.execute(plan_l,wait=should_wait)
    elif (move_group == group_r):
        arm = 'right'
        if(plan_r.joint_trajectory.points > 0):
	    success = group_r.execute(plan_r,wait=should_wait)
    else:
	arm = 'both'
	if(plan_both.joint_trajectory.points > 0):
	    success = group_both.execute(plan_both,wait=should_wait)
    if(success==1):    
	rospy.loginfo('moving ' + arm + ' arm success !')
    else:
        rospy.loginfo('moving ' + arm + ' arm failed !')
    return success

#Plan a trajectory and execute it
def plan_and_move(move_group, target,should_wait=True):
    plan(move_group,target)
    success = move(move_group,should_wait)
    return success

#Make a plan within the joint space
def plan_to_joints(move_group, positions):
    """Set joint values

    Plan the selected arm to make the joint positions match the given values

    :param positions: The desired joint values [j1-j7] (or [j1l-j7l,j1r-j7r] for both arms at once)
    :param arm: The selected arm (LEFT, RIGHT or BOTH)
    :type positions: float[]
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    global group_l
    global group_r
    global group_both
    global plan_l
    global plan_r
    global plan_both
    success = 0
    move_group.set_joint_value_target(positions)
    if (move_group == group_l):
        arm = 'left'
	plan_l = None
	plan_l = group_l.plan()
	if (plan_l.joint_trajectory.points > 0):
            rospy.loginfo('planning ' + arm + ' arm success !')
	    success = 1
        else:
            rospy.loginfo('planning ' + arm + ' arm failed !')
    elif (move_group == group_r):
        arm = 'right'
	plan_r = None
	plan_r = group_r.plan()
	if (plan_r.joint_trajectory.points > 0):
            rospy.loginfo('planning ' + arm + ' arm success !')
	    success = 1
        else:
            rospy.loginfo('planning ' + arm + ' arm failed !')
    else:
	arm = 'both'
	plan_both = None
	plan_both = group_both.plan()
	if (plan_both.joint_trajectory.points > 0):
            rospy.loginfo('planning ' + arm + ' arm success !')
	    success = 1
        else:
            rospy.loginfo('planning ' + arm + ' arm failed !')

    rospy.loginfo('Planning ' + arm + ' arm: Joint Position: {' + str(positions[0]) + ';' + str(positions[1]) + ';' + str(positions[2]) + ';' +
                                        str(positions[3]) + ';' + str(positions[4]) + ';' + str(positions[5]) + ';' + str(positions[6]) + '}.')
    return success

#Make a plan and move within the joint space
def plan_and_move_to_joints(move_group, positions,should_wait=True):
    """Set joint values

    Plans and Moves the selected arm to make the joint positions match the given values

    :param positions: The desired joint values [j1-j7] (or [j1l-j7l,j1r-j7r] for both arms at once)
    :param arm: The selected arm (LEFT, RIGHT or BOTH)
    :type positions: float[]
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    plan_to_joints(move_group,positions)
    success = move(move_group,should_wait)
    return success

#----------------------------HIGH LEVEL FUNCTIONS-----------------------------------#

def close_grippers(arm):
    """Closes the grippers.

    Closes the grippers with an effort of 15 and then relaxes the effort to 0.

    :param arm: The side to be closed (moveit_utils LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    global gripper_r
    global gripper_l
    success = 0
    last_gripper_r = -1
    last_gripper_l = -1
    if(arm == LEFT and gripper_l > 0.023):
	gripper_effort(LEFT, 15.0)
	rospy.sleep(0.4)
        gripper_effort(LEFT, 0.0)
	while (abs(last_gripper_l - gripper_l) > 0.0005):
            last_gripper_l = gripper_l
	    rospy.sleep(0.2)
	success = 1
    elif(arm == RIGHT and gripper_r > 0.023):
	gripper_effort(RIGHT, 15.0)
	rospy.sleep(0.4)
        gripper_effort(RIGHT, 0.0)
	while (abs(last_gripper_r - gripper_r) > 0.0005):
	    last_gripper_r = gripper_r
	    rospy.sleep(0.2)
	success = 1
    elif(arm == BOTH and (gripper_l > 0.023 or gripper_r > 0.023)):
	if(gripper_l > 0.023):
	    gripper_effort(LEFT, 15.0)
	if(gripper_r > 0.023):
	    gripper_effort(RIGHT, 15.0)
	rospy.sleep(0.4)
	gripper_effort(LEFT, 0.0)
	gripper_effort(RIGHT, 0.0)
	while (abs(last_gripper_r - gripper_r) > 0.0005 and abs(last_gripper_l - gripper_l) > 0.0005):
	    last_gripper_r = gripper_r
	    last_gripper_l = gripper_l
	    rospy.sleep(0.2)
	success = 1
    else:
	rospy.logerr('Unable to close gripper !')
    return success

def open_grippers(arm):
    """Opens the grippers.

    Opens the grippers with an effort of -15 and then relaxes the effort to 0.

    :param arm: The side to be opened (moveit_utils LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    global gripper_r
    global gripper_l
    success = 0
    last_gripper_r = -1
    last_gripper_l = -1
    if(arm == LEFT and gripper_l <= 0.023):
	gripper_effort(LEFT, -15.0)
        rospy.sleep(0.4)
        gripper_effort(LEFT, 0.0)
	while (abs(last_gripper_l - gripper_l) > 0.0005):
            last_gripper_l = gripper_l
	    rospy.sleep(0.2)
	success = 1
    elif(arm == RIGHT and gripper_r <= 0.023):
	gripper_effort(RIGHT, -15.0)
        rospy.sleep(0.4)
        gripper_effort(RIGHT, 0.0)
	while (abs(last_gripper_r - gripper_r) > 0.0005):
	    last_gripper_r = gripper_r
	    rospy.sleep(0.2)
	success = 1
    elif(arm == BOTH and (gripper_l <= 0.023 or gripper_r <= 0.023)):
	if(gripper_l <= 0.023):
	    gripper_effort(LEFT, -15.0)
	if(gripper_r <= 0.023):
	    gripper_effort(RIGHT, -15.0)
        rospy.sleep(0.4)
	gripper_effort(LEFT, 0.0)
	gripper_effort(RIGHT, 0.0)
	while (abs(last_gripper_r - gripper_r) > 0.0005 and abs(last_gripper_l - gripper_l) > 0.0005):
	    last_gripper_r = gripper_r
	    last_gripper_l = gripper_l
	    rospy.sleep(0.2)
	success = 1
    else:
	rospy.logerr('Unable to open gripper !')
    return success

#plan arm to the reset pose
def plan_home(arm):
    """Resets an arm

    Resets arm to its reset position

    :param arm: The selected arm (LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    global group_l
    global group_r
    global group_both
    success = 0
    #safeJointPositionR = [1.6379728317260742, 0.20191457867622375, -2.5927578258514404, 0.538416862487793, 2.7445449829101562, 1.5043296813964844, 1.7523150444030762]
    #safeJointPositionL = [-1.46564781665802, 0.3302380442619324, 2.507143497467041, 0.7764986753463745, -2.852548837661743, 1.659092664718628, 1.378138542175293]
    #safeJointPositionR = [0, -2.0689280275, -2.0561944901, 0.5235987755, 0, 0.6981317007, 0] #yumi home modified
    #safeJointPositionL = [0, -2.0689280275, 2.0561944901, 0.5235987755, 0, 0.6981317007, 0] #yumi home modified
    safeJointPositionR =[0.2139219045639038, -1.9016152620315552, -1.4134770631790161, 0.05151747167110443, 0.4104475677013397, 1.2984635829925537, 0.6140616536140442] #user study home
    #safeJointPositionR =[0.1444329023361206, -1.8475980758666992, -1.5281383991241455, 0.006057543680071831, 0.3669308125972748, 1.4296096563339233, 0.5392484068870544] #user study home
    safeJointPositionL =[-0.2139219045639038, -1.9016152620315552, 1.4134770631790161, 0.05151747167110443, -0.4104475677013397, 1.2984635829925537, -0.6140616536140442] #user study home
    
    if (arm == RIGHT):
        success = plan_to_joints(group_r,safeJointPositionR)
    elif (arm == LEFT):
        success = plan_to_joints(group_l,safeJointPositionL)
    elif (arm == BOTH):
	success = plan_to_joints(group_both,safeJointPositionL + safeJointPositionR)
    return success

#plan and move arm to the reset pose
def move_home(arm,should_wait=True):
    global group_l
    global group_r
    global group_both
    success = 0
    plan_home(arm)
    if (arm == RIGHT):
        success = move(group_r,should_wait)
    elif (arm == LEFT):
        success = move(group_l,should_wait)
    elif (arm == BOTH):
        success = move(group_both,should_wait)
	open_grippers(BOTH)
    return success

#move arm after planning
def move_arm(arm,should_wait=True):
    global group_l
    global group_r
    global group_both
    success = 0
    if (arm == RIGHT):
        success = move(group_r,should_wait)
    elif (arm == LEFT):
        success = move(group_l,should_wait)
    elif (arm == BOTH):
        success = move(group_both,should_wait)
    return success

#Wrapper for plan, just position, orientation and arm
def plan_simple(arm, pose):
    global group_l
    global group_r
    global group_both
    success = 0
    if (arm == LEFT):
        success = plan(group_l, pose)
    elif (arm == RIGHT):
        success = plan(group_r, pose)
    else:
    	success = plan(group_both, pose)
    return success

#Wrapper for plan_and_move, just position, orientation and arm
def move_simple(arm, pose,should_wait=True):
    global group_l
    global group_r
    global group_both
    success = 0
    if (arm == LEFT):
        success = plan_and_move(group_l, pose,should_wait)
    elif (arm == RIGHT):
        success = plan_and_move(group_r, pose,should_wait)
    else:
	success = plan_and_move(group_both, pose,should_wait)
    return success

#Wrapper for plan_and_move with cartesian path, just position, orientation and arm
def move_straight(arm, x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad):
    success = 0
    try:
        traverse_path(arm, [x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad], 10)
	success = 1
    except Exception:
        success = move_simple(arm, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))
    return success

#move on top of a point on the table and point at it
def move_point(arm, x_p, y_p, z_p=0.35, roll_rad=0, pitch_rad=PI, yaw_rad=0):
    global last_arm
    success = 0
    if(x_p<0.15 and x_p>0.55):
	rospy.logerr('X value is outside table area !')
	return success
    if(y_p<-0.6 and y_p>0.6):
	rospy.logerr('Y value is outside table area !')
	return success
    if (arm != BOTH):
        success = move_simple(arm, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))
	#success = move_straight(arm,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	#close_grippers(arm)
	last_arm = arm	
    else:
	if(y_p>=0):
	    #plan_home(RIGHT)
	    plan_simple(LEFT, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))
	    #success = move_arm(RIGHT)
	    success = move_arm(LEFT)
	    #close_grippers(LEFT)
            #success = move_straight(LEFT,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
    	    last_arm = LEFT
        else:
	    #plan_home(LEFT)
	    plan_simple(RIGHT, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))
    	    #success = move_arm(LEFT)
	    success = move_arm(RIGHT)
	    #close_grippers(RIGHT)
	    #success = move_straight(RIGHT,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	    last_arm = RIGHT    
    return success

#move on top of a point on the table and pick an object, adjust z_p to table height
def move_pick(arm, x_p, y_p, z_p=0.245, roll_rad=0, pitch_rad=PI, yaw_rad=0):
    global last_arm
    success = 0
    if(x_p<0.15 and x_p>0.55):
	rospy.logerr('X value is outside table area !')
	return success
    if(y_p<-0.6 and y_p>0.6):
	rospy.logerr('Y value is outside table area !')
	return success
    if (arm == LEFT):
	open_grippers(LEFT)
        success = move_straight(LEFT,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(LEFT, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))
	close_grippers(LEFT)
        success = move_straight(LEFT,x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(LEFT, create_pose_euler(x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad))
    elif(arm == RIGHT):
	open_grippers(RIGHT)
        success = move_straight(RIGHT,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(RIGHT, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))	
	close_grippers(RIGHT)
        success = move_straight(RIGHT,x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(RIGHT, create_pose_euler(x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad))
    else:
	open_grippers(last_arm)
        success = move_straight(last_arm,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(last_arm, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))	
	close_grippers(last_arm)
        success = move_straight(last_arm,x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(last_arm, create_pose_euler(x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad))
    return success

#move on top of a point on the table and place an object, adjust z_p to table height
def move_place(arm, x_p, y_p, z_p=0.245, roll_rad=0, pitch_rad=PI, yaw_rad=0):
    global last_arm
    success = 0
    if(x_p<0.15 and x_p>0.55):
	rospy.logerr('X value is outside table area !')
	return success
    if(y_p<-0.6 and y_p>0.6):
	rospy.logerr('Y value is outside table area !')
	return success
    if (arm == LEFT):
        success = move_straight(LEFT,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(LEFT, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))
	open_grippers(LEFT)
        success =move_straight(LEFT,x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(LEFT, create_pose_euler(x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad))
    elif(arm == RIGHT):
        success = move_straight(RIGHT,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(RIGHT, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))	
	open_grippers(RIGHT)
        success = move_straight(RIGHT,x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(RIGHT, create_pose_euler(x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad))
    else:
        success = move_straight(last_arm,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(last_arm, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))	
	open_grippers(last_arm)
        success = move_straight(last_arm,x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad)
	#success = move_simple(last_arm, create_pose_euler(x_p, y_p, 0.35, roll_rad, pitch_rad, yaw_rad))	
    return success

#move on top of a point on the table, and pick an object
def move_point_pick(arm, x_p, y_p, z_p=0.35, roll_rad=0, pitch_rad=PI, yaw_rad=0):
    success = 0
    success = move_point(arm, x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
    success = move_pick(arm, x_p, y_p, 0.245, roll_rad, pitch_rad, yaw_rad)   
    return success

#move on top of a point on the table, and pick an object, and handover
def move_point_pick_handover(arm, x_p, y_p, z_p=0.35, roll_rad=0, pitch_rad=PI, yaw_rad=0):
    success = 0
    success = move_point(arm, x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
    success = move_pick(arm, x_p, y_p, 0.245, roll_rad, pitch_rad, yaw_rad)   
    y_h = 0.35
    if (arm == RIGHT):
	y_h = -y_h
    success = move_handover_ready(arm,0.7,y_h,z_p,HALF_PI,0,HALF_PI)
    return success

#move on top of a point on the table, and place an object
def move_point_place(arm, x_p, y_p, z_p=0.35, roll_rad=0, pitch_rad=PI, yaw_rad=0):
    success = 0
    success = move_point(arm, x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
    success = move_place(arm, x_p, y_p, 0.245, roll_rad, pitch_rad, yaw_rad)   
    return success


#move to the hand over area (where the human hand is, 10 cm behind in the x direction)
def move_handover_ready(arm, x_p=0.7 , y_p, z_p=0.35, roll_rad=HALF_PI, pitch_rad=0, yaw_rad=HALF_PI):
    global last_arm
    success = 0
    if(x_p<0.25 and x_p>0.75):
	rospy.logerr('X value is outside table area !')
	return success
    if(y_p<-0.4 and y_p>0.4):
	rospy.logerr('Y value is outside table area !')
	return success
    if(z_p<0.2 and z_p>0.6):
	rospy.logerr('Z value is outside table area !')
	return success
    if (arm != BOTH):
        success = move_simple(arm, create_pose_euler(x_p-0.1, y_p, z_p, roll_rad, pitch_rad, yaw_rad))
	last_arm = arm	
    else:
	if(y_p>=0):
	    #plan_home(RIGHT)
	    plan_simple(LEFT, create_pose_euler(x_p-0.1, y_p, z_p, roll_rad, pitch_rad, yaw_rad))
	    #success = move_arm(RIGHT)
	    success = move_arm(LEFT)
	    #close_grippers(LEFT)
    	    last_arm = LEFT
        else:
	    #plan_home(LEFT)
	    plan_simple(RIGHT, create_pose_euler(x_p-0.1, y_p, z_p, roll_rad, pitch_rad, yaw_rad))
    	    #success = move_arm(LEFT)
	    success = move_arm(RIGHT)
	    #close_grippers(RIGHT)
	    last_arm = RIGHT    
    return success

#give object to human hand
def move_handover_give(arm, x_p , y_p, z_p=0.4, roll_rad=HALF_PI, pitch_rad=0, yaw_rad=HALF_PI):
    global last_arm
    success = 0
    if(x_p<0.25 and x_p>0.55):
	rospy.logerr('X value is outside table area !')
	return success
    if(y_p<-0.4 and y_p>0.4):
	rospy.logerr('Y value is outside table area !')
	return success
    if(z_p<0.2 and z_p>0.6):
	rospy.logerr('Z value is outside table area !')
	return success
    if (arm == LEFT):
        success = move_straight(LEFT,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	open_grippers(LEFT)
        success = move_straight(LEFT,x_p-0.1, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
    elif(arm == RIGHT):
	success = move_straight(RIGHT,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	open_grippers(RIGHT)
        success = move_straight(RIGHT,x_p-0.1, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
    else:
        success = move_straight(last_arm,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	open_grippers(last_arm)
        success = move_straight(last_arm,x_p-0.1, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
    return success

#take object from human hand
def move_handover_take(arm, x_p , y_p, z_p=0.4, roll_rad=HALF_PI, pitch_rad=0, yaw_rad=HALF_PI):
    global last_arm
    success = 0 
    if(x_p<0.25 and x_p>0.55):
	rospy.logerr('X value is outside table area !')
	return success
    if(y_p<-0.4 and y_p>0.4):
	rospy.logerr('Y value is outside table area !')
	return success
    if(z_p<0.2 and z_p>0.6):
	rospy.logerr('Z value is outside table area !')
	return success
    if (arm == LEFT):
	success = move_straight(LEFT,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	close_grippers(LEFT)
        success = move_straight(LEFT,x_p-0.1, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
    elif(arm == RIGHT):
	success = move_straight(RIGHT,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	close_grippers(RIGHT)
        success = move_straight(RIGHT,x_p-0.1, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
    else:
	success = move_straight(last_arm,x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
	close_grippers(last_arm)
        success = move_straight(last_arm,x_p-0.1, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
    return success
