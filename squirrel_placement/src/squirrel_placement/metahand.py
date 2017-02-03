import rospy
import actionlib
from actionlib_msgs.msg import *
from squirrel_manipulation_msgs.msg import DropAction, DropGoal, DropResult, PutDownAction, PutDownGoal, PutDownResult, PtpAction, PtpGoal
from kclhand_control.msg import ActuateHandAction, ActuateHandGoal
from visualization_msgs.msg import Marker

class MetaHand(object):
    def __init__(self):
        while rospy.get_time() == 0.0:
            pass

        rospy.loginfo(rospy.get_caller_id() + ': starting up')
        self.metahand = actionlib.SimpleActionClient('hand_controller/actuate_hand', ActuateHandAction)
        self.metahand.wait_for_server()
        self.ptp = actionlib.SimpleActionClient('cart_ptp', PtpAction)
        self.ptp.wait_for_server()
        self.drop_server = actionlib.SimpleActionServer('metahand_drop_server',
                                                        DropAction,
                                                        execute_cb=self._execute_drop,
                                                        auto_start=False)
        self.put_server = actionlib.SimpleActionServer('metahand_place_server',
                                                        PutDownAction,
                                                        execute_cb=self._execute_placement,
                                                        auto_start=False)

        self.drop_result = DropResult()
        self.put_result = PutDownResult()
        self.drop_server.start()
        self.put_server.start()
        self.markerPub = rospy.Publisher('put_downMarker', Marker, queue_size=10)
        rospy.loginfo(rospy.get_caller_id() + ': started')


    def _execute_drop(self, goal):
        rospy.loginfo(rospy.get_caller_id() + ': called')

        if self.drop_server.is_preempt_requested():
            rospy.loginfo(rospy.get_caller_id() + ': preempted')
            self.drop_server.set_preempted()
            return

        # open hand
        open_hand = ActuateHandGoal()
        open_hand.command = 0
        rospy.loginfo("Opening hand...")
        self.metahand.send_goal(open_hand)
        self.metahand.wait_for_result()
        if self.metahand.get_state() == GoalStatus.ABORTED:
            error = 'Could not open hand.'
            self.drop_server.set_aborted(self.drop_result, error)
            return

        # we're done
        success = 'Object released.'
        self.drop_server.set_succeeded(self.drop_result, success)
        return


    def _execute_placement(self, goal):
        rospy.loginfo(rospy.get_caller_id() + ': called')

        if self.put_server.is_preempt_requested():
            rospy.loginfo(rospy.get_caller_id() + ': preempted')
            self.put_server.set_preempted()
            return

        self._visualize_put_down(goal.destPoseSE2)

        correct_pose = None
        if not goal.destPoseSE2.header.frame_id == 'origin':
            goal.destPoseSE2.header.stamp = self.tf_listener.getLatestCommonTime(goal.destPoseSE2.header.frame_id, 'origin')
            correct_pose = self.tf_listener.transformPose('origin', goal.destPoseSE2)
        else:
            correct_pose = goal.destPoseSE2.pose

        ptp_goal = PtpGoal()
        ptp_goal.pose.position.x = correct_pose.position.x
        ptp_goal.pose.position.y = correct_pose.position.y
        ptp_goal.pose.position.z = correct_pose.position.z
        ptp_goal.pose.orientation.w = correct_pose.orientation.w
        ptp_goal.pose.orientation.x = correct_pose.orientation.x
        ptp_goal.pose.orientation.y = correct_pose.orientation.y
        ptp_goal.pose.orientation.z = correct_pose.orientation.z

        rospy.loginfo("Approaching placement pose...")
        self.ptp.send_goal(ptp_goal)
        self.ptp.wait_for_result()
        result = self.ptp.get_result()
        rospy.loginfo(result.result_status)
        if result.result_status == "Execution failed.":
            error = 'Approaching placement pose failed.'
            self.put_server.set_aborted(self.put_result, error)
            return

        # open hand
        open_hand = ActuateHandGoal()
        open_hand.command = 0
        rospy.loginfo("Opening hand...")
        self.metahand.send_goal(open_hand)
        self.metahand.wait_for_result()
        if self.metahand.get_state() == GoalStatus.ABORTED:
            error = 'Could not open hand.'
            self.drop_server.set_aborted(self.drop_result, error)
            return

        # we're done
        success = 'Object placed and released.'
        self.put_server.set_succeeded(self.put_result, success)
        return

    def _visualize_put_down(self, pose):
        put_downMarker = Marker()
        put_downMarker.header.frame_id = "/origin"
        put_downMarker.header.stamp = rospy.get_rostime()
        put_downMarker.ns = "grasp"
        put_downMarker.id = 0
        put_downMarker.type = 2
        put_downMarker.action = 0
        put_downMarker.pose.position = pose.position
        put_downMarker.pose.orientation.x = 0
        put_downMarker.pose.orientation.y = 0
        put_downMarker.pose.orientation.z = 0
        put_downMarker.pose.orientation.w = 1.0
        put_downMarker.scale.x = 0.05
        put_downMarker.scale.y = 0.05
        put_downMarker.scale.z = 0.05
        put_downMarker.color.r = 0.0
        put_downMarker.color.g = 1.0
        put_downMarker.color.b = 0.0
        put_downMarker.color.a = 1.0
        put_downMarker.lifetime = rospy.Duration(secs=20)
        self.markerPub.publish(put_downMarker)
