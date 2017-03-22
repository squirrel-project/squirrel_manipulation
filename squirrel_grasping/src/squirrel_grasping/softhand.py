import rospy
import actionlib
from squirrel_manipulation_msgs.msg import PtpAction, PtpGoal
from squirrel_manipulation_msgs.srv import SoftHandGrasp
from geometry_msgs.msg import Pose, PoseStamped
from squirrel_manipulation_msgs.msg import BlindGraspAction
from squirrel_manipulation_msgs.msg import BlindGraspResult
from squirrel_manipulation_msgs.msg import BlindGraspFeedback
from visualization_msgs.msg import Marker

import tf
import time
from copy import copy

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
        self.server = actionlib.SimpleActionServer('softhand_grasp_server',
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
            correct_pose.pose.position.z+=d
            pre_grasp = copy(correct_pose)
        else:
            correct_pose = goal.heap_center_pose
            correct_pose.pose.position.z+=d
            pre_grasp = copy(correct_pose)

        ptp_pre_goal = PtpGoal()
        ptp_pre_goal.pose.position.x = pre_grasp.pose.position.x
        ptp_pre_goal.pose.position.y = pre_grasp.pose.position.y
        ptp_pre_goal.pose.position.z = pre_grasp.pose.position.z+0.2
        #do not change values below
        ptp_pre_goal.pose.orientation.w = 0.73
        ptp_pre_goal.pose.orientation.x = 0.0
        ptp_pre_goal.pose.orientation.y = 0.69
        ptp_pre_goal.pose.orientation.z = 0.0

        ptp_goal = PtpGoal()
        ptp_goal.pose.position.x = correct_pose.pose.position.x
        ptp_goal.pose.position.y = correct_pose.pose.position.y
        ptp_goal.pose.position.z = correct_pose.pose.position.z

        '''
           x: 0.340830731389
        y: 0.619075146426
        z: -0.341226694197
    w: 0.619794093861

        '''
        #do not change values below
        ptp_goal.pose.orientation.w = 0.73
        ptp_goal.pose.orientation.x = 0.0
        ptp_goal.pose.orientation.y = 0.69
        ptp_goal.pose.orientation.z = 0.0

        self._visualize_grasp(ptp_goal)

        # open hand
        rospy.loginfo("Opening hand...")
        self.softhand(0.0)
        time.sleep(3)


        # moving to pre pose
        rospy.loginfo("Approaching pre pose...")
        self.ptp.send_goal(ptp_pre_goal)
        self.ptp.wait_for_result()
        result = self.ptp.get_result()
        rospy.loginfo(result.result_status)
        if result.result_status == "Execution failed.":
            error = 'Approaching pre pose failed.'
            self.server.set_aborted(self.grasp_result, error)
            return


        # move to grasp pose
        rospy.loginfo("Approaching grasp pose...")
        self.ptp.send_goal(ptp_goal)
        self.ptp.wait_for_result()
        result = self.ptp.get_result()
        rospy.loginfo(result.result_status)
        if result.result_status == "Execution failed.":
            error = 'Approaching grasp pose failed.'
            self.server.set_aborted(self.grasp_result, error)
            return


        #grasp the object
        rospy.loginfo("Grasping...")
        self.softhand(0.8)
        time.sleep(3)


        # retract the arm
        rospy.loginfo("Retracting...")
        self.ptp.send_goal(ptp_pre_goal)
        self.ptp.wait_for_result()
        result = self.ptp.get_result()
        rospy.loginfo(result.result_status)
        if result.result_status == "Execution failed.":
            error = 'Retracting the arm failed.'
            self.server.set_aborted(self.grasp_result, error)
            return


        # we're done
        success = 'Object grasped.'
        self.server.set_succeeded(self.grasp_result, success)
        return


    def _visualize_grasp(self, ptp_goal):
        graspMarker = Marker()
        graspMarker.header.frame_id = "/origin"
        graspMarker.header.stamp = rospy.get_rostime()
        graspMarker.ns = "grasp"
        graspMarker.id = 0
        graspMarker.type = 2
        graspMarker.action = 0
        graspMarker.pose.position = ptp_goal.pose.position
        graspMarker.pose.orientation.x = 0
        graspMarker.pose.orientation.y = 0
        graspMarker.pose.orientation.z = 0
        graspMarker.pose.orientation.w = 1.0
        graspMarker.scale.x = 0.05
        graspMarker.scale.y = 0.05
        graspMarker.scale.z = 0.05
        graspMarker.color.r = 0.0
        graspMarker.color.g = 1.0
        graspMarker.color.b = 0.0
        graspMarker.color.a = 1.0
        graspMarker.lifetime = rospy.Duration(secs=30)
        self.markerPub.publish(graspMarker)

