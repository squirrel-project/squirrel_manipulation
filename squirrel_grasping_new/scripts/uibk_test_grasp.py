#! /usr/bin/env python

import sys
from copy import deepcopy
from squirrel_manipulation_msgs.msg import BlindGraspGoal, BlindGraspAction, DropAction, DropGoal, PutDownAction, PutDownGoal
import rospy
import actionlib
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Facing down configuration is different to TUW

if __name__ == '__main__':
    rospy.init_node('test_grasp', anonymous=True)
   
    print('Test started...')
    
    client =  actionlib.SimpleActionClient('squirrel_object_manipulation_server', BlindGraspAction)
    client.wait_for_server()

    grasp_goal = BlindGraspGoal()
    # !!! Currently using this string to change the command type (open, close, grasp, place)
    # Need to update message definition
    grasp_goal.object_id = 'grasp'
    grasp_goal.heap_center_pose.header.stamp = rospy.Time.now()
    grasp_goal.heap_center_pose.header.frame_id = 'map'  # Always plan in map frame??
    
    # Facing down (orientation works for KCL hand...not sure about Innsbruck hand)
    grasp_goal.heap_center_pose.pose.position.x = 0.38
    grasp_goal.heap_center_pose.pose.position.y = 0.17
    grasp_goal.heap_center_pose.pose.position.z = 0.25
    grasp_goal.heap_center_pose.pose.orientation.x = -0.707
    grasp_goal.heap_center_pose.pose.orientation.y = 0.00
    grasp_goal.heap_center_pose.pose.orientation.z = 0.00
    grasp_goal.heap_center_pose.pose.orientation.w = 0.707
    # http://quaternions.online/
    # Euler: x = -90, y = 0, z = 0
    # Adjust z (-ve direction to not be exactly straight with axis)
    
    # Ignoring all other information (kept here just in case)
    # alternate pose
    grasp_goal.heap_center_pose_static.header.stamp = rospy.Time.now()
    grasp_goal.heap_center_pose_static.header.frame_id = 'map'
    grasp_goal.heap_center_pose_static.pose.position.x = grasp_goal.heap_center_pose.pose.position.x + 0.1
    grasp_goal.heap_center_pose_static.pose.position.y = grasp_goal.heap_center_pose.pose.position.y + 0.1
    grasp_goal.heap_center_pose_static.pose.position.z = 0.5
    grasp_goal.heap_center_pose_static.pose.orientation.x = 0.0 #0.0
    grasp_goal.heap_center_pose_static.pose.orientation.y = 0.0 #0.0
    grasp_goal.heap_center_pose_static.pose.orientation.z = 0.0 #0.0
    grasp_goal.heap_center_pose_static.pose.orientation.w = 1.0 #1.0
    
    grasp_goal.heap_bounding_cylinder.diameter = 0.2
    grasp_goal.heap_bounding_cylinder.height = 0.08
    
    # Send the goal to the server
    #rospy.loginfo('Sending grasp goal:\n{0}'.format(grasp_goal))
    client.send_goal(grasp_goal)
    rospy.loginfo('Waiting for completion...')
    client.wait_for_result()
    rospy.loginfo('Complete.')
    rospy.loginfo('Returned state: ' + str(client.get_state()))
    
    rospy.loginfo('Done.')
