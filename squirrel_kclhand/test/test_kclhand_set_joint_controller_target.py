#!/usr/bin/env python
import roslib; roslib.load_manifest('squirrel_kclhand')
import rospy

import actionlib
import yaml
from squirrel_kclhand_msgs.msg import *

test_set_joint_controller_target_goal = open("test/testdata_kclhand_set_joint_controller_target", 'r')

def set_joint_controller_target_client():
    client = actionlib.SimpleActionClient('/set_joint_controller_target', SetJointTargetAction)
    client.wait_for_server()
    
    goal =SetJointTargetGoal()
    genpy.message.fill_message_args(goal, yaml.load(test_set_joint_controller_target_goal))

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


if __name__ == "__main__":
    rospy.init_node('set_joint_controller_target_test')
    print set_joint_controller_target_client()
