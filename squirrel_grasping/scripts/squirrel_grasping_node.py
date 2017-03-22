#! /usr/bin/env python

import rospy

from squirrel_grasping import softhand
from squirrel_grasping import metahand


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_grasping_node')
        hand = rospy.get_param('~hand')
        gripper = None

        if hand == 'softhand':
            gripper = softhand.SoftHand()
        elif hand == 'kclhand':
            gripper = metahand.MetaHand()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
