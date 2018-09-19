#! /usr/bin/env python

from face_output.srv import *
import rospy

# Brings in the SimpleActionClient
import actionlib

import homer_tts.msg

client = None

smileys_dict = {
        0: ".",     #rest
        1: ":)",    #happy
        2: ":(",    #sad
        3: ":&",    #afraid
        4: ":O",    #surprised
        5: ">:",    #angry
        6: ":!"     #disgusted
    }

expressions_dict = {
        0: "rest",
        1: "happy",
        2: "sad",
        3: "afraid",
        4: "surprised",
        5: "angry",
        6: "disgusted"
    }

def run():
    """Starts the node

    Runs to start the node and initialize everthing. Runs forever via Spin()

    :returns: Nothing
    :rtype: None
    """
    global client

    rospy.init_node('face_output_server')

    # Creates the SimpleActionClient, passing the type of the action
    # (Speak) to the constructor.
    client = actionlib.SimpleActionClient('speak', homer_tts.msg.SpeakAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    
    rospy.Service('face/say_and_express', FaceInterface, handle_say_and_express)

    print "                             Face Output Server is running.. "
    rospy.spin()

def handle_say_and_express(req):
    print ".                                    say_and_express - request received!"
    print ".                                    Say: " + req.text + " , with " + expressions_dict.get(req.emotion, "rest") + " expression"
    print ".                                    Executing request..."
    global client

    smileys = smileys_dict.get(req.emotion, ".")

    # Creates a goal to send to the action server.
    goal = homer_tts.msg.SpeakGoal(text=smileys + " " + req.text)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    result = client.wait_for_result(rospy.Duration(10, 0))

    if (result):
        print "         say_and_express - request executed !"
    else:
        print "         say_and_express - execution failed !"

    return FaceInterfaceResponse(result)

if __name__ == '__main__':
    try:
        run()
        print "####################################     Face Output Server Terminated     ####################################"
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
