import rospy
import actionlib

from squirrel_manipulation_msgs.msg import BlindGraspAction

class BlindGraspServer:
    def __init__(self):
        while rospy.get_time() == 0.0: pass
        rospy.loginfo(rospy.get_caller_id() + ': starting BlindGraspServer')
        self.server = actionlib.SimpleActionServer('blind_grasp', BlindGraspAction, self.execute, False)
        self.server.start()

    
    def execute(self, goal):
        rospy.loginfo(rospy.get_caller_id() + ': Requested %s', goal)
        self.server.set_succeeded()




