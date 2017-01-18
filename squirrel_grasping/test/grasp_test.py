#! /usr/bin/env python

from squirrel_manipulation_msgs.msg import BlindGraspGoal, BlindGraspAction
import rospy
import actionlib

if __name__ == '__main__':
    rospy.init_node('grasp_test')
   
    print('test started...')
    
    #client =  actionlib.SimpleActionClient('kclhand_grasp_server', BlindGraspAction)
    client =  actionlib.SimpleActionClient('softhand_grasp_server', BlindGraspAction)

    client.wait_for_server()
    print('server found...')

    goal = BlindGraspGoal()
    goal.object_id = 'test'
    goal.heap_center_pose.header.stamp = rospy.Time.now()
    goal.heap_center_pose.header.frame_id = 'origin'
    goal.heap_center_pose.pose.position.x = 0.4
    goal.heap_center_pose.pose.position.y = 0.4
    goal.heap_center_pose.pose.position.z = 0.2
    goal.heap_center_pose.pose.orientation.x = 0.0
    goal.heap_center_pose.pose.orientation.y = 0.0
    goal.heap_center_pose.pose.orientation.z = 0.0
    goal.heap_center_pose.pose.orientation.w = 1.0
    goal.heap_bounding_cylinder.diameter = 0.2
    goal.heap_bounding_cylinder.height = 0.3

    print('sending goal:')
    print(goal)

    client.send_goal(goal)
    print('waiting for completion')
    client.wait_for_result()
    
    print('done')





    
