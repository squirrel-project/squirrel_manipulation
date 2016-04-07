#! /usr/bin/env python                                                                                                                                                                                                                                             

from moveit_commander import PlanningSceneInterface
import threading

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

import rospy


def emulate():
    pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)
    rate = rospy.Rate(10) 

    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [3, 3, 0.1]

    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = -0.0501
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    co = CollisionObject()
    co.header.frame_id = '/base_link'
    co.id = 'ground'
    co.primitives = [primitive]
    co.primitive_poses = [pose]
    co.operation = co.ADD

    while not rospy.is_shutdown():
        co.header.stamp = rospy.Time.now()
        pub.publish(co)
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('moveit_ground_emulation')
        emulate()
    except rospy.ROSInterruptException:
        pass
