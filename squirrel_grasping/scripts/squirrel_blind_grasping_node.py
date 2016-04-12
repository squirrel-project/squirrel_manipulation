#! /usr/bin/env python

import rospy

from squirrel_grasping.server.blindGraspServer import BlindGraspServer


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_blind_grasping_node')
        BGS = BlindGraspServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
