#! /usr/bin/env python

import sys
from copy import deepcopy
from squirrel_manipulation_msgs.msg import ManipulationGoal, ManipulationAction
import rospy
import actionlib
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Facing down configuration is different to TUW

if __name__ == '__main__':
    rospy.init_node('test_manipulation', anonymous=True)
   
    print('Test started...')
    
    client =  actionlib.SimpleActionClient('squirrel_object_manipulation_server', ManipulationAction)
    client.wait_for_server()

    manipulation_goal = ManipulationGoal()
    manipulation_goal.manipulation_type = 'grasp'
    manipulation_goal.pose.header.stamp = rospy.Time.now()
    manipulation_goal.pose.header.frame_id = 'map'  # Always plan in map frame??
    
    # Define the pose of the object
    manipulation_goal.pose.pose.position.x = 0.38
    manipulation_goal.pose.pose.position.y = 0.17
    manipulation_goal.pose.pose.position.z = 0.17
    
    # This is now the pose of the object! (gripper pose taken care of inside object manipulation server)
    manipulation_goal.pose.pose.orientation.x = 0.5
    manipulation_goal.pose.pose.orientation.y = 0.5
    manipulation_goal.pose.pose.orientation.z = -0.5
    manipulation_goal.pose.pose.orientation.w = 0.5
   
    # Size of the object (height is used to find position of gripper for grasping)
    manipulation_goal.object_bounding_cylinder.diameter = 0.2
    manipulation_goal.object_bounding_cylinder.height = 0.08
    
    # Testing moving to joint configuration
    manipulation_goal.joints = [0, 0, 0, 0, 0, 0, 0, 0]
    
    # Send the goal to the server
    #rospy.loginfo('Sending manipulation goal:\n{0}'.format(manipulation_goal))
    client.send_goal(manipulation_goal)
    rospy.loginfo('Waiting for completion...')
    client.wait_for_result()
    rospy.loginfo('Complete.')
    rospy.loginfo('Returned state: ' + str(client.get_state()))
    
    rospy.loginfo('Done.')
