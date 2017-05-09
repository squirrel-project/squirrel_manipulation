import rospy
import actionlib
from actionlib_msgs.msg import *
from squirrel_manipulation_msgs.msg import DropAction, DropGoal, DropResult, PutDownAction, PutDownGoal, PutDownResult
from squirrel_manipulation_msgs.msg import PtpAction, PtpGoal, JointPtpAction, JointPtpGoal
from kclhand_control.msg import ActuateHandAction, ActuateHandGoal
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
import tf

class MetaHand(object):
    def __init__(self):
        while rospy.get_time() == 0.0:
            pass

        rospy.loginfo(rospy.get_caller_id() + ': starting up')
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.metahand = actionlib.SimpleActionClient('hand_controller/actuate_hand', ActuateHandAction)
        self.metahand.wait_for_server()
        self.ptp = actionlib.SimpleActionClient('cart_ptp', PtpAction)
        self.ptp.wait_for_server()
	self.joint_ptp = actionlib.SimpleActionClient('joint_ptp', JointPtpAction)
        self.joint_ptp.wait_for_server()
	rospy.Subscriber("/real/robotino/joint_control/get_state", JointState, self._joints_callback)
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


    def _joints_callback(self, data):
	self.joint_state = data
    
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

        correct_pose = None
        if not goal.destPoseSE2.header.frame_id == 'origin':
            goal.destPoseSE2.header.stamp = self.tf_listener.getLatestCommonTime(goal.destPoseSE2.header.frame_id, 'origin')
            correct_pose = self.tf_listener.transformPose('origin', goal.destPoseSE2).pose
        else:
            correct_pose = goal.destPoseSE2.pose

        self._visualize_put_down(correct_pose)

        ptp_goal = PtpGoal()
        ptp_goal.pose.position.x = correct_pose.position.x
        ptp_goal.pose.position.y = correct_pose.position.y
        ptp_goal.pose.position.z = correct_pose.position.z
	if ptp_goal.pose.position.z < 0.2:
		ptp_goal.pose.position.z = 0.2
        #ptp_goal.pose.orientation.w = correct_pose.orientation.w
        #ptp_goal.pose.orientation.x = correct_pose.orientation.x
        #ptp_goal.pose.orientation.y = correct_pose.orientation.y
        #ptp_goal.pose.orientation.z = correct_pose.orientation.z

	goal.destPoseSE2.pose.orientation.w = -0.072 #-0.408
        goal.destPoseSE2.pose.orientation.x = -0.300 #0.896
        goal.destPoseSE2.pose.orientation.y = -0.451 #0.154
        goal.destPoseSE2.pose.orientation.z = 0.838 #0.088

        rospy.loginfo("Approaching placement pose...")
        #self.ptp.send_goal(ptp_goal)
        #self.ptp.wait_for_result()
        #result = self.ptp.get_result()
	joint_ptp_goal = JointPtpGoal()
	base_z = self.joint_state.position[2] + 0.75 # add 80 degree rotation to the put down
	if base_z > 3.14:
		base_z = 3.14
		#base_z = self.joint_state.position[2]
	rospy.loginfo('Adjusted z rotation from ' + str(self.joint_state.position[2]) + ' to' + str(base_z))
	# [x,y,z, 1.0685184933389604, 1.0723638027469542, -0.48548916231515227, 0.74918607207707, -0.15201119653169812]
	#joint_ptp_goal.joints.data = (self.joint_state.position[0], self.joint_state.position[1], base_z, 1.069, 1.072, -0.485, 1.072, -0.485, 0.749, -0.152)
	joint_ptp_goal.joints.data = (self.joint_state.position[0], self.joint_state.position[1], base_z, 1.069, 1.072, -0.485, 0.749, -0.152)
	print joint_ptp_goal
	self.joint_ptp.send_goal(joint_ptp_goal)
	self.joint_ptp.wait_for_result()
	result = self.joint_ptp.get_result()
        #rospy.loginfo(result.result_status)
        #if result.result_status == "execution failed.":
        #    error = 'Approaching placement pose failed.'
            #self.put_server.set_aborted(self.put_result, error)
            #return

        # open hand
        open_hand = ActuateHandGoal()
        open_hand.command = 0
        rospy.loginfo("Opening hand...")
        self.metahand.send_goal(open_hand)
        self.metahand.wait_for_result()
        if self.metahand.get_state() == GoalStatus.ABORTED:
            error = 'Could not open hand.'
	    rospy.logerr(error)
            #self.drop_server.set_aborted(self.drop_result, error)
            #return

        # we're done
        success = 'Object placed and released.'
        self.put_server.set_succeeded(self.put_result, success)

	# Return to up position
        ptp_goal.pose.position.z += 0.2
        rospy.loginfo("Retracting...")
        #self.ptp.send_goal(ptp_goal)
        #self.ptp.wait_for_result()
        #result = self.ptp.get_result()
        #rospy.loginfo(result.result_status)
        #if result.result_status == "execution failed.":
        #    error = 'Retraction failed.'
        #    self.put_server.set_aborted(self.put_result, error)
        #    return
	joint_ptp_goal = JointPtpGoal()
        joint_ptp_goal.joints.data = [self.joint_state.position[0], self.joint_state.position[1], self.joint_state.position[2], 0.7, 1.6, 0.00, -1.7, -1.8]
        rospy.loginfo("Retracting...")
        self.joint_ptp.send_goal(joint_ptp_goal)
        self.joint_ptp.wait_for_result()
        result = self.joint_ptp.get_result()
        rospy.loginfo(result.result_status)
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
