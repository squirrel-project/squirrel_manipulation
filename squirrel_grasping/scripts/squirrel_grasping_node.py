#! /usr/bin/env python

import rospy

from squirrel_grasping.server.graspServer import GraspServer


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_grasping_node')
        BGS = GraspServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
