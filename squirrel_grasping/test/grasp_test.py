#! /usr/bin/env python

import sys
from squirrel_manipulation_msgs.msg import BlindGraspGoal, BlindGraspAction, DropAction, DropGoal, PutDownAction, PutDownGoal
from kclhand_control.msg import ActuateHandAction, ActuateHandGoal
import rospy
import actionlib

if __name__ == '__main__':
    rospy.init_node('grasp_test')
   
    print('test started...')
    
    client =  actionlib.SimpleActionClient('metahand_grasp_server', BlindGraspAction)
    drop = actionlib.SimpleActionClient('metahand_drop_server', DropAction)
    place = actionlib.SimpleActionClient('metahand_place_server', PutDownAction)
    metahand = actionlib.SimpleActionClient('hand_controller/actuate_hand', ActuateHandAction) 
 
    #client = actionlib.SimpleActionClient('softhand_grasp_server', BlindGraspAction)
    #drop = actionlib.SimpleActionClient('softhand_drop_server', DropAction)
    #place = actionlib.SimpleActionClient('softhand_place_server', PutDownAction)

    client.wait_for_server()
    drop.wait_for_server()
    place.wait_for_server()
    metahand.wait_for_server()
    rospy.loginfo('servers found...')

    grasp_goal = BlindGraspGoal()
    grasp_goal.object_id = 'test'
    grasp_goal.heap_center_pose.header.stamp = rospy.Time.now()
    grasp_goal.heap_center_pose.header.frame_id = 'map'
    grasp_goal.heap_center_pose.pose.position.x = 1.78
    grasp_goal.heap_center_pose.pose.position.y = 0.369
    #grasp_goal.heap_center_pose.header.frame_id = 'map'
    #grasp_goal.heap_center_pose.pose.position.x = 2.08
    #grasp_goal.heap_center_pose.pose.position.y = 0.136
    grasp_goal.heap_center_pose.pose.position.z = 0.0
    grasp_goal.heap_center_pose.pose.orientation.x = 0.0 #0.0
    grasp_goal.heap_center_pose.pose.orientation.y = 0.0 #0.0
    grasp_goal.heap_center_pose.pose.orientation.z = 0.0 #0.0
    grasp_goal.heap_center_pose.pose.orientation.w = 1.0 #1.0
    #alternate pose
    grasp_goal.heap_center_pose_static.header.stamp = rospy.Time.now()
    grasp_goal.heap_center_pose_static.header.frame_id = 'map'
    grasp_goal.heap_center_pose_static.pose.position.x = grasp_goal.heap_center_pose.pose.position.x + 0.1
    grasp_goal.heap_center_pose_static.pose.position.y = grasp_goal.heap_center_pose.pose.position.y + 0.1
    #grasp_goal.heap_center_pose.header.frame_id = 'map'
    #grasp_goal.heap_center_pose.pose.position.x = 2.08
    #grasp_goal.heap_center_pose.pose.position.y = 0.136
    grasp_goal.heap_center_pose_static.pose.position.z = 0.0
    grasp_goal.heap_center_pose_static.pose.orientation.x = 0.0 #0.0
    grasp_goal.heap_center_pose_static.pose.orientation.y = 0.0 #0.0
    grasp_goal.heap_center_pose_static.pose.orientation.z = 0.0 #0.0
    grasp_goal.heap_center_pose_static.pose.orientation.w = 1.0 #1.0
    
    grasp_goal.heap_bounding_cylinder.diameter = 0.2
    grasp_goal.heap_bounding_cylinder.height = 0.08
    open_hand = ActuateHandGoal()
    
    open_hand.command = 0
    open_hand.force_limit = 1

    rospy.loginfo('Sending grasp goal:\n{0}'.format(grasp_goal))
    client.send_goal(grasp_goal)
    rospy.loginfo('Waiting for completion...')
    client.wait_for_result()
    rospy.loginfo('Complete.')
    rospy.loginfo('Returned state: ' + str(client.get_state()))
    
    #ch = raw_input('Press y to drop the object?')
    #if ch != 'y':
    #    rospy.logerr('Aborted by user.')
    #    sys.exit(2)
    
    #metahand.send_goal(open_hand)
    #metahand.wait_for_result()
    
    choice = raw_input("Enter 'd' for drop and 'p' for placement: ")

    if choice == 'd':    
        rospy.loginfo('Dropping.')
        drop_goal = DropGoal()
        drop_goal.destination_id = 'somewhere'
        rospy.loginfo('Sending drop goal:\n{0}'.format(drop_goal))
        rospy.loginfo(drop_goal)
        drop.send_goal(drop_goal)
        rospy.loginfo('Waiting for completion...')
        drop.wait_for_result()
        rospy.loginfo('Complete.')

    elif choice == 'p':
        rospy.loginfo('Placing')
        put_down_goal = PutDownGoal()
        put_down_goal.destination_id = 'somewhere'
        put_down_goal.destPoseSE2.header.frame_id = 'origin'
        #put_down_goal.destPoseSE2.pose.position.x = 0.4
        #put_down_goal.destPoseSE2.pose.position.y = 0.4
        #put_down_goal.destPoseSE2.pose.position.z = 0.3
        #put_down_goal.destPoseSE2.pose.orientation.w = 1.0
        #put_down_goal.destPoseSE2.pose.orientation.x = 0.0
        #put_down_goal.destPoseSE2.pose.orientation.y = 0.0
        #put_down_goal.destPoseSE2.pose.orientation.z = 0.0
        put_down_goal.destPoseSE2.pose.position.x = 0.30
        put_down_goal.destPoseSE2.pose.position.y = -0.515
        put_down_goal.destPoseSE2.pose.position.z = 0.2
        put_down_goal.destPoseSE2.pose.orientation.w = -0.408
        put_down_goal.destPoseSE2.pose.orientation.x = 0.896
        put_down_goal.destPoseSE2.pose.orientation.y = 0.154
        put_down_goal.destPoseSE2.pose.orientation.z = 0.088
        
        rospy.loginfo('Sending put down goal:\n{0}'.format(put_down_goal))
        place.send_goal(put_down_goal)
        rospy.loginfo('Waiting for completion...')
        place.wait_for_result()
        rospy.loginfo('Complete.')

    else:
        rospy.logerr('Unknown choice: {0}'.format(choice))
    
    rospy.loginfo('Done.')





    
