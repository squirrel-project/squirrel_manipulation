import rospy
import actionlib
from squirrel_manipulation_msgs.msg import PtpAction, PtpGoal
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
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.softhand = rospy.ServiceProxy('softhand_grasp', SoftHandGrasp)
        self.ptp = actionlib.SimpleActionClient('cart_ptp', PtpAction)
        self.ptp.wait_for_server()
        self._server = actionlib.SimpleActionServer('softhand_grasp_server', 
                                                    BlindGraspAction, 
                                                    execute_cb=self._execute_grasp, 
                                                    auto_start=False)
        self._server.start()
        rospy.loginfo(rospy.get_caller_id() + ': started')


    def _execute_grasp(self, goal):
        rospy.loginfo(rospy.get_caller_id() + ': called')
       
        if self._server.is_preempt_requested():
            rospy.loginfo(rospy.get_caller_id() + ': preempted')
            self._server.set_preempted()
            return

        correct_pose = None
        d = goal.heap_bounding_cylinder.height/2.0

        if not goal.heap_center_pose.header.frame_id == 'origin':
            pose = goal.heap_center_pose
            pose.header.stamp = self.tf_listener.getLatestCommonTime(goal.heap_center_pose.header.frame_id, 'origin') 
            correct_pose = self.tf_listener.transformPose('origin', goal.heap_center_pose)
            correct_pose.pose.position.z+=(d+0.05)
        else:
            correct_pose = goal.heap_center_pose
            correct_pose.pose.position.z+=(d+0.05)
        
        ptp_goal = PtpGoal()
        ptp_goal.pose.position.x = correct_pose.pose.position.x
        ptp_goal.pose.position.y = correct_pose.pose.position.y
        ptp_goal.pose.position.z = correct_pose.pose.position.z
        ptp_goal.pose.orientation.w = 0.73
        ptp_goal.pose.orientation.x = 0.0
        ptp_goal.pose.orientation.y = 0.69
        ptp_goal.pose.orientation.z = 0.0

        # open hand
        rospy.loginfo("Opening hand...")
        self.softhand(0.0)

        # move to grasp pose
        rospy.loginfo("Approaching grasp pose...")
        self.ptp.send_goal(ptp_goal)
        self.ptp.wait_for_result()
        result = self.ptp.get_result()                
        rospy.loginfo(result.result_status)
        if result.result_status == "execution failed.":
            error = 'Approaching grasp pose failed.'
            self.server.set_aborted(self.grasp_result, error)
            return
            
        #grasp the object
        rospy.loginfo("Grasping...")
        time.sleep(1)
        softhand(0.8)
            
        # retract the arm
        ptp_goal.pose.position.z = goal.heap_center_pose.pose.position.z + d + 0.3
        rospy.loginfo("Retracting...")
        self.ptp.send_goal(goal)
        self.ptp.wait_for_result()
        result = self.ptp.get_result()                
        rospy.loginfo(result.result_status)
        if result.result_status == "execution failed.":
            error = 'Retracting the arm failed.'
            self.server.set_aborted(self.grasp_result, error)
            return

        # we're done
        success = 'Object grasped.'
        self.server.set_succeeded(self.grasp_result, success)
        return


