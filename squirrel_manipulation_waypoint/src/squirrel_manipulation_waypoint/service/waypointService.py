import rospy
import random
import numpy as np

from squirrel_waypoint_msgs.srv import ManipulationWaypointService, ManipulationWaypointServiceResponse
from geometry_msgs.msg import Pose, PoseWithCovariance
from tf.transformations import euler_from_quaternion


class WaypointService:
    def __init__(self):
        while rospy.get_time() == 0.0: pass
        rospy.loginfo(rospy.get_caller_id() + ': starting WaypointService')

        self.waypoints = rospy.Service('/squirrel_manipulation_waypoint', ManipulationWaypointService, self.compute)


    def compute(self, msg):
        #mock
        random.seed()
        rospy.loginfo(rospy.get_caller_id() + ': request: %s', msg)
        reply = ManipulationWaypointServiceResponse()
        for i in range(0,3):            
            pose = PoseWithCovariance()
            pose.pose.position.x = random.uniform(0,1)
            pose.pose.position.y = random.uniform(0,1)
            pose.pose.position.z = random.uniform(0,1)
            pose.pose.orientation.x = random.uniform(0,1)
            pose.pose.orientation.y = random.uniform(0,1)
            pose.pose.orientation.z = random.uniform(0,1)
            pose.pose.orientation.w = random.uniform(0,1)
            [x, y, z] = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            pose.covariance = np.cov(np.array([[pose.pose.position.x], [pose.pose.position.y], [pose.pose.position.z], [x], [y], [z]])).flatten()
            reply.poses.append(pose)            
        reply.weights = [0.1, 0.4, 0.5]
        return reply




