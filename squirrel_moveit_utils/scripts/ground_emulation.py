#! /usr/bin/env python                                                                                                                                                                                                                                             

from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
#from moveit_msgs.msg import CollisionObject
#from shape_msgs.msg import SolidPrimitive

import threading
import rospy


if __name__ == '__main__':

  rospy.init_node('load_scene')
  while rospy.get_time() == 0.0: 
    pass

  psi = PlanningSceneInterface()
  rospy.sleep(1.0)

  rospy.wait_for_service("/get_planning_scene")
  rospy.sleep(5.0)
	### Add a floor

    #psi = PlanningSceneInterface()
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
  touch_links = ["wheel0_link","wheel1_link","wheel2_link"]
  psi.attach_box("base_link", "ground", pose, (3, 3, 0.1), touch_links)


#def emulate():
#    pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)
#    rate = rospy.Rate(10) 
#
#    primitive = SolidPrimitive()
#    primitive.type = primitive.BOX
#    primitive.dimensions = [3, 3, 0.1]
#
#    pose = Pose()
#    pose.position.x = 0
#    pose.position.y = 0
#    pose.position.z = -0.1
#    pose.orientation.x = 0
#    pose.orientation.y = 0
#    pose.orientation.z = 0
#    pose.orientation.w = 1

#    co = CollisionObject()
#    co.header.frame_id = '/base_link'
#    co.id = 'ground'
#    co.primitives = [primitive]
#    co.primitive_poses = [pose]
#    co.operation = co.ADD

#    while not rospy.is_shutdown():
#        co.header.stamp = rospy.Time.now()
#        pub.publish(co)
#        rate.sleep()


#if __name__ == '__main__':
#    try:
#        rospy.init_node('moveit_ground_emulation')
#        emulate()
#    except rospy.ROSInterruptException:
#        pass

