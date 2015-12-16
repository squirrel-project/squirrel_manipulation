#! /usr/bin/env python

import rospy

from squirrel_kinesthetic_teaching.teacher.kinestheticTeacher import KinestheticTeacher


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_kinesthetic_teaching_node')
        KT = KinestheticTeacher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
