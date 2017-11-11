#! /usr/bin/env python

import sys
from squirrel_manipulation_msgs.msg import BlindGraspGoal, BlindGraspAction, DropAction, DropGoal, PutDownAction, PutDownGoal
import rospy
import actionlib

if __name__ == '__main__':
    rospy.init_node('test_grasp_server')
   
    print('Test started...')
    
    client =  actionlib.SimpleActionClient('squirrel_grasp_server', BlindGraspAction)
    client.wait_for_server()
    rospy.loginfo('Servers found...')

    grasp_goal = BlindGraspGoal()
    grasp_goal.object_id = 'test'
    grasp_goal.heap_center_pose.header.stamp = rospy.Time.now()
    grasp_goal.heap_center_pose.header.frame_id = 'map'
    grasp_goal.heap_center_pose.pose.position.x = 1.78
    grasp_goal.heap_center_pose.pose.position.y = 0.369
    grasp_goal.heap_center_pose.pose.position.z = 0.5
    grasp_goal.heap_center_pose.pose.orientation.x = 0.0 #0.0
    grasp_goal.heap_center_pose.pose.orientation.y = 0.0 #0.0
    grasp_goal.heap_center_pose.pose.orientation.z = 0.0 #0.0
    grasp_goal.heap_center_pose.pose.orientation.w = 1.0 #1.0

    #alternate pose
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

    #rospy.loginfo('Sending grasp goal:\n{0}'.format(grasp_goal))
    client.send_goal(grasp_goal)
    rospy.loginfo('Waiting for completion...')
    client.wait_for_result()
    rospy.loginfo('Complete.')
    rospy.loginfo('Returned state: ' + str(client.get_state()))
    
    rospy.loginfo('Done.')





    
