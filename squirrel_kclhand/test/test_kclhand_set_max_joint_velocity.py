#!/usr/bin/env python
import roslib; roslib.load_manifest('squirrel_kclhand')
import rospy

import actionlib
import yaml
from squirrel_kclhand_msgs.msg import *

test_set_max_joint_velocity_goal = open("test/testdata_kclhand_set_max_joint_velocity", 'r')

def set_max_joint_velocity_client():
    client = actionlib.SimpleActionClient('/set_max_joint_velocity', SetMaxJointVelocityAction)
    client.wait_for_server()
    
    goal =SetMaxJointVelocityGoal()
    genpy.message.fill_message_args(goal, yaml.load(test_set_max_joint_velocity_goal))

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


if __name__ == "__main__":
    rospy.init_node('set_max_joint_velocity_test')
    print set_max_joint_velocity_client()
