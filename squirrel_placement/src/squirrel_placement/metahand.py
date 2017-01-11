import rospy
import actionlib
from actionlib_msgs.msg import *
from squirrel_manipulation_msgs.msg import DropAction, DropGoal, DropResult, PutDownAction, PutDownGoal, PutDownResult, PtpAction, PtpGoal
from kclhand_control.msg import ActuateHandAction, ActuateHandGoal

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

        rospy.loginfo("Approaching placement pose...")
        self.ptp.send_goal(goal.destPoseSE2)
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
