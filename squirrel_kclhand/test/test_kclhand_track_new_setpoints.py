#!/usr/bin/env python
import roslib; roslib.load_manifest('squirrel_kclhand')
import rospy

import actionlib
import yaml
from squirrel_kclhand_msgs.msg import *

test_track_new_setpoints_goal = open("test/testdata_kclhand_track_new_setpoints", 'r')

def track_new_setpoints_client():
    client = actionlib.SimpleActionClient('/track_new_setpoints', TrackNewSetpointsAction)
    client.wait_for_server()
    
    goal =TrackNewSetpointsGoal()
    genpy.message.fill_message_args(goal, yaml.load(test_track_new_setpoints_goal))

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


if __name__ == "__main__":
    rospy.init_node('track_new_setpoints_test')
    print track_new_setpoints_client()
