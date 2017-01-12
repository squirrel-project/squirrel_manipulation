import rospy
import actionlib
from actionlib_msgs.msg import *
from squirrel_manipulation_msgs.msg import PtpAction, PtpGoal
from kclhand_control.msg import ActuateHandAction, ActuateHandGoal
from squirrel_manipulation_msgs.msg import BlindGraspAction
from squirrel_manipulation_msgs.msg import BlindGraspResult
from squirrel_manipulation_msgs.msg import BlindGraspFeedback
from visualization_msgs.msg import Marker

import tf
import time
from copy import copy

class MetaHand(object):
    def __init__(self):
        while rospy.get_time() == 0.0:
            pass

        rospy.loginfo(rospy.get_caller_id() + ': starting up')
        self.metahand = actionlib.SimpleActionClient('hand_controller/actuate_hand', ActuateHandAction)
        self.metahand.wait_for_server()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.ptp = actionlib.SimpleActionClient('cart_ptp', PtpAction)
        self.ptp.wait_for_server()
        self.server = actionlib.SimpleActionServer('metahand_grasp_server',
                                                    BlindGraspAction,
                                                    execute_cb=self._execute_grasp,
                                                    auto_start=False)
        self.grasp_result = BlindGraspResult()
        self.server.start()
        self.markerPub = rospy.Publisher('graspMarker', Marker, queue_size=10)
        rospy.loginfo(rospy.get_caller_id() + ': started')


    def _execute_grasp(self, goal):
        rospy.loginfo(rospy.get_caller_id() + ': called')

        if self.server.is_preempt_requested():
            rospy.loginfo(rospy.get_caller_id() + ': preempted')
            self.server.set_preempted()
            return

        pre_grasp = None
        correct_pose = None
        d = goal.heap_bounding_cylinder.height/2.0

        if not goal.heap_center_pose.header.frame_id == 'origin':
            goal.heap_center_pose.header.stamp = self.tf_listener.getLatestCommonTime(goal.heap_center_pose.header.frame_id, 'origin')
            correct_pose = self.tf_listener.transformPose('origin', goal.heap_center_pose)
            correct_pose.pose.position.z+=(d+0.11)
            pre_grasp = copy(correct_pose)
        else:
            correct_pose = goal.heap_center_pose
            correct_pose.pose.position.z+=(d+0.11)
            pre_grasp = copy(correct_pose)

        ptp_pre_goal = PtpGoal()
        ptp_pre_goal.pose.position.x = pre_grasp.pose.position.x
        ptp_pre_goal.pose.position.y = pre_grasp.pose.position.y
        ptp_pre_goal.pose.position.z = pre_grasp.pose.position.z+0.2
        #do not change values below
        ptp_pre_goal.pose.orientation.w = 0.287
        ptp_pre_goal.pose.orientation.x = 0.912
        ptp_pre_goal.pose.orientation.y = 0.290
        ptp_pre_goal.pose.orientation.z = 0.028

        ptp_goal = PtpGoal()
        ptp_goal.pose.position.x = correct_pose.pose.position.x
        ptp_goal.pose.position.y = correct_pose.pose.position.y
        ptp_goal.pose.position.z = correct_pose.pose.position.z
        #do not change values below
        ptp_goal.pose.orientation.w = 0.287
        ptp_goal.pose.orientation.x = 0.912
        ptp_goal.pose.orientation.y = 0.290
        ptp_goal.pose.orientation.z = 0.028

        open_hand = ActuateHandGoal()
        open_hand.command = 0
        close_hand = ActuateHandGoal()
        close_hand.command = 1
        close_hand.grasp_type = 0

        self._visualize_grasp(ptp_goal)

        # open hand
        rospy.loginfo("Opening hand...")
        self.metahand.send_goal(open_hand)
        self.metahand.wait_for_result()
        if self.metahand.get_state() == GoalStatus.ABORTED:
            error = 'Could not open hand.'
            self.server.set_aborted(self.grasp_result, error)
            return

        ##########################
        print ptp_pre_goal
        ch = raw_input('Do you want to continue (y)?')
        if ch  != 'y':
            error = 'Aborted by user.'
            self.server.set_aborted(self.grasp_result, error)
            return
        ##########################

        # move to pre pose
        rospy.loginfo("Approaching pre pose...")
        self.ptp.send_goal(ptp_pre_goal)
        self.ptp.wait_for_result()
        result = self.ptp.get_result()
        rospy.loginfo(result.result_status)
        if result.result_status == "execution failed.":
            error = 'Approaching pre pose failed.'
            self.server.set_aborted(self.grasp_result, error)
            return

        ##########################
        print ptp_goal
        ch = raw_input('Do you want to continue (y)?')
        if ch  != 'y':
            error = 'Aborted by user.'
            self.server.set_aborted(self.grasp_result, error)
            return
        ##########################

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

        # grasp the object
        rospy.loginfo("Grasping...")
        self.metahand.send_goal(open_hand)
        result = self.metahand.wait_for_result()
        if self.metahand.get_state() == GoalStatus.ABORTED:
            error = 'Grasping failed.'
            self.server.set_aborted(self.grasp_result, error)
            return

        # retract the arm
        ptp_goal.pose.position.z+=0.3

        ##########################
        print ptp_goal
        ch = raw_input('Do you want to continue (y)?')
        if ch  != 'y':
            error = 'Aborted by user.'
            self.server.set_aborted(self.grasp_result, error)
            return
        ##########################

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


    def _visualize_grasp(self, ptp_goal):
        self.graspMarker = Marker()
        self.graspMarker.header.frame_id = "/origin"
        self.graspMarker.header.stamp = rospy.get_rostime()
        self.graspMarker.ns = "grasp"
        self.graspMarker.id = 0
        self.graspMarker.type = 2
        self.graspMarker.action = 0
        self.graspMarker.pose.position = ptp_goal.pose.position
        self.graspMarker.pose.orientation.x = 0
        self.graspMarker.pose.orientation.y = 0
        self.graspMarker.pose.orientation.z = 0
        self.graspMarker.pose.orientation.w = 1.0
        self.graspMarker.scale.x = 1.0
        self.graspMarker.scale.y = 1.0
        self.graspMarker.scale.z = 1.0

        self.graspMarker.color.r = 0.0
        self.graspMarker.color.g = 1.0
        self.graspMarker.color.b = 0.0
        self.graspMarker.color.a = 1.0

        self.graspMarker.lifetime = rospy.Duration(secs=5)

        self.markerPub.publish(graspMarker)

