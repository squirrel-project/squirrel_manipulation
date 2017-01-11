#! /usr/bin/env python

import rospy

from squirrel_placement import softhand
from squirrel_placement import metahand


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_placement_node')
        hand = rospy.get_param('~hand')
        gripper = None

        if hand == 'softhand':
            gripper = softhand.SoftHand()
        elif hand == 'kclhand':
            gripper = metahand.MetaHand()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
