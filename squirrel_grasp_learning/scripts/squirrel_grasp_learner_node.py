#! /usr/bin/env python

import rospy

from squirrel_grasp_learning.learner import MHGraspLearner


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_grasp_learning_node')
        BGS = MHGraspLearner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
