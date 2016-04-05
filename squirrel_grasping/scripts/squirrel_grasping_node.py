#! /usr/bin/env python

import rospy

from squirrel_grasping.server.graspServer import GraspServer
from squirrel_grasping.server.graspServer import BlindGraspServer


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_grasping_node')
        BGS = BlindGraspServer()
        #GS = GraspServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
