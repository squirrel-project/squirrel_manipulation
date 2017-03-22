#!/usr/bin/python

import rospy
import tf

def getLumpPose():
  #rospy.init_node('mytest')

  listener = tf.TransformListener()


  lump_pose = False
  while not lump_pose:
    try:
      (trans,rot) = listener.lookupTransform('/base_link', '/myobject', rospy.Time(0))
      lump_pose = True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue

    return trans


if __name__ == "__main__":
  print getLumpPose()
	

