#! /usr/bin/env python

import rospy

from squirrel_grasping.server.hafGraspServer import HAFGraspServer


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_haf_grasping_node')
        HGS = HAFGraspServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
