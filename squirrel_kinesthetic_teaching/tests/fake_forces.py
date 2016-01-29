#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, EmptyRequest
import random


def setup():
    rospy.init_node('force_faker', anonymous=True)
    random.seed()


def start():
    rospy.loginfo('starting teacher')
    rospy.wait_for_service('/squirrel_manipulation/start_teaching')
    try:
        start_ = rospy.ServiceProxy('/squirrel_manipulation/start_teaching', Empty)
        start_(EmptyRequest())
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%e    

def stop():
    rospy.loginfo('stopping teacher')
    rospy.wait_for_service('/squirrel_manipulation/stop_teaching')
    try:
        stop_ = rospy.ServiceProxy('/squirrel_manipulation/stop_teaching', Empty)
        stop_(EmptyRequest())
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%e    


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
    setup()
    start()
    chat()
    stop()

