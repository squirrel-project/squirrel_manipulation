#! /usr/bin/env python

import rospy

#from squirrel_grasp_learning.learner import MHGraspLearner
from squirrel_grasp_learning.learner import test


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_grasp_learning_node')
        #MHGL = MHGraspLearner()
        tester = test.Test()
        tester.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
