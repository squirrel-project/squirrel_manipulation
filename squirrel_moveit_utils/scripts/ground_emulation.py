#! /usr/bin/env python                                                                                                                                                                                                                                                          

from moveit_commander import roscpp_initialize, roscpp_shutdown, MoveGroupCommander, PlanningSceneInterface
import threading

import rospy


def add_ground():
    psi = PlanningSceneInterface()
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    # offset such that the box is 0.1 mm below ground (to prevent collision with the robot itself)                                                                                                                                                                            
    pose.pose.position.z = -0.0501
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    pose.header.stamp = rospy.get_rostime()
    pose.header.frame_id = "base_link"
    psi.attach_box("base_link", "ground", pose, (3, 3, 0.1))


if __name__ == '__main__':
    try:
        rospy.init_node('moveit_ground_emulation')
        add_ground()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
