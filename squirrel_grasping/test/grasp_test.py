#! /usr/bin/env python

from squirrel_manipulation_msgs.msg import BlindGraspGoal, BlindGraspAction, DropAction, DropGoal, PutDownAction, PutDownGoal
import rospy
import actionlib

if __name__ == '__main__':
    rospy.init_node('grasp_test')
   
    print('test started...')
    
    #client =  actionlib.SimpleActionClient('metahand_grasp_server', BlindGraspAction)
    #drop = actionlib.SimpleActionClient('metahand_drop_server', DropAction)
    #place = actionlib.SimpleActionClient('metahand_place_server', PutDownAction)
 
    client = actionlib.SimpleActionClient('softhand_grasp_server', BlindGraspAction)
    drop = actionlib.SimpleActionClient('softhand_drop_server', DropAction)
    place = actionlib.SimpleActionClient('softhand_place_server', PutDownAction)

    client.wait_for_server()
    drop.wait_for_server()
    place.wait_for_server()
    rospy.loginfo('servers found...')


    grasp_goal = BlindGraspGoal()
    grasp_goal.object_id = 'test'
    grasp_goal.heap_center_pose.header.stamp = rospy.Time.now()
    grasp_goal.heap_center_pose.header.frame_id = 'origin'
    grasp_goal.heap_center_pose.pose.position.x = 0.4
    grasp_goal.heap_center_pose.pose.position.y = 0.4
    grasp_goal.heap_center_pose.pose.position.z = 0.05
    grasp_goal.heap_center_pose.pose.orientation.x = 0.0
    grasp_goal.heap_center_pose.pose.orientation.y = 0.0
    grasp_goal.heap_center_pose.pose.orientation.z = 0.0
    grasp_goal.heap_center_pose.pose.orientation.w = 1.0
    grasp_goal.heap_bounding_cylinder.diameter = 0.2
    grasp_goal.heap_bounding_cylinder.height = 0.3

    rospy.loginfo('Sending grasp goal:\n{0}'.format(grasp_goal))
    client.send_goal(grasp_goal)
    rospy.loginfo('Waiting for completion...')
    client.wait_for_result()
    rospy.loginfo('Complete.')

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
        put_down_goal.destPoseSE2.position.x = 0.5
        put_down_goal.destPoseSE2.position.y = 0.5
        put_down_goal.destPoseSE2.position.z = 0.4
        put_down_goal.destPoseSE2.orientation.w = 0.73
        put_down_goal.destPoseSE2.orientation.x = 0.0
        put_down_goal.destPoseSE2.orientation.y = 0.69
        put_down_goal.destPoseSE2.orientation.z = 0.0
        
        rospy.loginfo('Sending put down goal:\n{0}'.format(put_down_goal))
        place.send_goal(put_down_goal)
        rospy.loginfo('Waiting for completion...')
        place.wait_for_result()
        rospy.loginfo('Complete.')

    else:
        rospy.logerr('Unknown choice: {0}'.format(choice))
    
    rospy.loginfo('Done.')





    
