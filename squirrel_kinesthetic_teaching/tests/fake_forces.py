#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import random


def setup():
    rospy.init_node('force_faker', anonymous=True)
    random.seed()


def chat():
    rospy.loginfo('chatting...')
    rate = rospy.Rate(0.25)
    pub = rospy.Publisher('/wrist', Float64MultiArray, queue_size=10)
    while not rospy.is_shutdown():
        forces = Float64MultiArray()
        forces.data.append(random.uniform(-1, -0.9))
        forces.data.append(random.uniform(-1, -0.9))
        forces.data.append(random.uniform(-1, -0.9))
        forces.data.append(random.uniform(0.1, 0.15))
        forces.data.append(random.uniform(0.1, 0.15))
        forces.data.append(random.uniform(0.1, 0.15))
        rospy.loginfo(forces)
        pub.publish(forces)
        rate.sleep()


if __name__ == '__main__':
    chat()

