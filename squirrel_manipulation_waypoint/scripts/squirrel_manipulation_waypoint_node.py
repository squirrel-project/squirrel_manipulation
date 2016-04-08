#! /usr/bin/env python

import rospy

from squirrel_manipulation_waypoint.service.waypointService import WaypointService


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_manipulation_waypoint_node')
        WS = WaypointService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
