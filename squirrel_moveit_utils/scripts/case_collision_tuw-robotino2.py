#! /usr/bin/env python                                                                                                                                                                                                                                             

from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from tf import transformations
import rospkg
#from moveit_msgs.msg import CollisionObject
#from shape_msgs.msg import SolidPrimitive

import threading
import rospy


if __name__ == '__main__':

  rospy.init_node('load_scene_case')
  while rospy.get_time() == 0.0: 
    pass

  psi = PlanningSceneInterface()
  rospy.sleep(1.0)

  rospy.wait_for_service("/get_planning_scene")
  rospy.sleep(5.0)
	### Add a case

  pose = PoseStamped()
  pose.pose.position.x = 0
  pose.pose.position.y = 0
    # offset such that the box is 0.1 mm below ground (to prevent collision with the robot itself)
  pose.pose.position.z = 0
  pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = transformations.quaternion_from_euler( 0,0,-0.3 )
  #pose.pose.orientation.x = 0
  #pose.pose.orientation.y = 0
  #pose.pose.orientation.z = 0
  #pose.pose.orientation.w = 1
  pose.header.stamp = rospy.get_rostime()
  pose.header.frame_id = "base_link"
  touch_links = ["base_link","kinect_link","profile_link"]
  rospack = rospkg.RosPack()
  rospath = rospack.get_path('robotino_description')
  filename = rospath + "/meshes/case.stl"
  psi.attach_mesh("base_link", "case", pose, filename , (1, 1, 1), touch_links)
  #psi.add_mesh("case", pose, filename , (1, 1, 1))
  #psi.attach_box("base_link", "case", pose, (1, 1, 0.1), touch_links)

