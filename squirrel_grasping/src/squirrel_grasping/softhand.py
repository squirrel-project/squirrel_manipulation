import rospy
import actionlib
from squirrel_manipulation_msgs.msg import PtpAction, PtpGoal, JointPtpAction, JointPtpGoal
from squirrel_manipulation_msgs.srv import SoftHandGrasp
from geometry_msgs.msg import Pose, PoseStamped
from squirrel_manipulation_msgs.msg import BlindGraspAction
from squirrel_manipulation_msgs.msg import BlindGraspResult
from squirrel_manipulation_msgs.msg import BlindGraspFeedback

import tf

import time, sys

class SoftHand(object):
    def __init__(self):
        while rospy.get_time() == 0.0:
            pass

        rospy.loginfo(rospy.get_caller_id() + ': starting up')
        rospy.wait_for_service('softhand_grasp')
        self.softhand = rospy.ServiceProxy('softhand_grasp', SoftHandGrasp)
        self.ptp = actionlib.SimpleActionClient('cart_ptp', PtpAction)
        self.ptp.wait_for_server()
        self._server = actionlib.SimpleActionServer('pisaGrasp', 
                                                    BlindGraspAction, 
                                                    execute_cb=self._execute_grasp, 
                                                    auto_start=False)
        rospy.loginfo(rospy.get_caller_id() + ': started')


    def _execute_grasp(self, goal):
        rospy.loginfo(rospy.get_caller_id() + ': called')
        
        if self._server.is_preempt_requested():
            rospy.loginfo(rospy.get_caller_id() + ': preempted')
            self._server.set_preempted()
            return
        
        goal = PtpGoal()
        goal.pose.position.x = goal.heap_center_pose.pose.position.x
        goal.pose.position.y = goal.heap_center_pose.pose.position.y
        goal.pose.position.z = goal.heap_center_pose.pose.position.z + d + 0.1
        goal.pose.orientation.w = 0.73
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.69
        goal.pose.orientation.z = 0.0

        rospy.loginfo("Open hand.")
        self.softhand(0.0)

        self.ptp.send_goal(goal)
        self.ptp.wait_for_result()
        result = self.ptp.get_result()                
        rospy.loginfo(result.result_status)
        if result.result_status == "execution failed.":
            sys.exit("Grasping failed.")
            
        rospy.loginfo("Grasping.")
        time.sleep(1)
        softhand(0.8)
            
        goal.pose.position.z = goal.heap_center_pose.pose.position.z + d + 0.3
        rospy.loginfo("Retract.")
        self.ptp.send_goal(goal)
        self.ptp.wait_for_result()
        result = self.ptp.get_result()                
        rospy.loginfo(result.result_status)
        if result.result_status == "execution failed.":
            sys.exit("Retractiong failed.")
