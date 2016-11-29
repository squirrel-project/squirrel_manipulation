#!/usr/bin/env python

from squirrel_manipulation_msgs.msg import BlindGraspGoal, BlidnGraspAction
import rospy
import actionlib

rospy.init_node('grasp_test')
rospy.spin()

client =  actionlib.SimpleActionClient('softhand_grasp_server', BlindGraspAction)

goal = BlindGraspGoal()
goal.object_id = 'test'
goal.heap_center_pose.header.stamp = rospy.get_time()
goal.heap_center_pose.header.frame_id = 'origin'
goal.heap_center_pose.pose.position.x = 0.4
goal.heap_center_pose.pose.position.y = 0.4
goal.heap_center_pose.pose.position.z = 0.4
goal.heap_center_pose.pose.orientation.x = 0.0
goal.heap_center_pose.pose.orientation.y = 0.0
goal.heap_center_pose.pose.orientation.z = 0.0
goal.heap_center_pose.pose.orientation.w = 1.0
goal.heap_bounding_cylinder.diameter = 0.2
goal.heap_bounding_cylinder.height = 0.3

client.send_goal(goal)
result = client.wait_for_result()
print result





    
