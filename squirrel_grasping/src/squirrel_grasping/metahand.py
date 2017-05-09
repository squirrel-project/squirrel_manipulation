import rospy
import actionlib
from actionlib_msgs.msg import *
from squirrel_manipulation_msgs.msg import PtpAction, PtpGoal, JointPtpAction, JointPtpGoal
from kclhand_control.msg import ActuateHandAction, ActuateHandGoal
from squirrel_manipulation_msgs.msg import BlindGraspAction
from squirrel_manipulation_msgs.msg import BlindGraspResult
from squirrel_manipulation_msgs.msg import BlindGraspFeedback
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import numpy as np
import tf
import time
from copy import copy
from time import sleep

class MetaHand(object):
    def __init__(self):
        while rospy.get_time() == 0.0:
            pass

        rospy.loginfo(rospy.get_caller_id() + ': starting up')
        self.metahand = actionlib.SimpleActionClient('hand_controller/actuate_hand', ActuateHandAction)
        rospy.loginfo('Waiting for hand_controller/actuate_hand')
        self.metahand.wait_for_server()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.ptp = actionlib.SimpleActionClient('cart_ptp', PtpAction)
	self.joint_ptp = actionlib.SimpleActionClient('joint_ptp', JointPtpAction)
        rospy.loginfo('Waiting for ptp server')
        self.ptp.wait_for_server()
	rospy.Subscriber("/real/robotino/joint_control/get_state", JointState, self._joints_callback)
        self.server = actionlib.SimpleActionServer('metahand_grasp_server',
                                                    BlindGraspAction,
                                                    execute_cb=self._execute_grasp,
                                                    auto_start=False)
        self.grasp_result = BlindGraspResult()
        self.server.start()
        self.markerPub = rospy.Publisher('graspMarker', Marker, queue_size=10)
        rospy.loginfo(rospy.get_caller_id() + ': started')

    def _joints_callback(self, data):
        self.joint_state = data

    def _execute_grasp(self, goal):
        rospy.loginfo(rospy.get_caller_id() + ': called')

        rospy.loginfo(rospy.get_caller_id() + ': visualizing static and computed poses')
        self._visualize_poses(goal)
        rospy.loginfo(rospy.get_caller_id() + ': continuing as usual')

        if self.server.is_preempt_requested():
            rospy.loginfo(rospy.get_caller_id() + ': preempted')
            self.server.set_preempted()
            return

        pre_grasp = None
        correct_pose = None
        correct_pose_static = None
        d = goal.heap_bounding_cylinder.height/2
	if d < 0:
		d = 0.13
        rospy.loginfo("goal.heap_bounding_cylinder.height = " + str(goal.heap_bounding_cylinder.height))
        rospy.loginfo("d = " + str(d))
    	rospy.loginfo("received frame " + str(goal.heap_center_pose.header.frame_id))

        #computed pose
        if not goal.heap_center_pose.header.frame_id == 'origin':
            goal.heap_center_pose.header.stamp = self.tf_listener.getLatestCommonTime(goal.heap_center_pose.header.frame_id, 'origin')
            correct_pose = self.tf_listener.transformPose('origin', goal.heap_center_pose)
            correct_pose.pose.position.z = d+0.1
            pre_grasp = copy(correct_pose)
        else:
            correct_pose = goal.heap_center_pose
            correct_pose.pose.position.z = d+0.1
            pre_grasp = copy(correct_pose)

        #static pose
        if not goal.heap_center_pose_static.header.frame_id == 'origin':
            goal.heap_center_pose_static.header.stamp = self.tf_listener.getLatestCommonTime(goal.heap_center_pose_static.header.frame_id, 'origin')
            correct_pose_static = self.tf_listener.transformPose('origin', goal.heap_center_pose_static)
            correct_pose_static.pose.position.z = d+0.1
        else:
            correct_pose_static = goal.heap_center_pose_static
            correct_pose_static.pose.position.z = d+0.1

        #if too far off, take static pose values
        if np.abs(correct_pose.pose.position.x - correct_pose_static.pose.position.x) > 0.15:
            rospy.loginfo("replacing correct_pose.pose.position.x")
            correct_pose.pose.position.x = correct_pose_static.pose.position.x
        if np.abs(correct_pose.pose.position.y - correct_pose_static.pose.position.y) > 0.15:
            rospy.loginfo("replacing correct_pose.pose.position.y")
            correct_pose.pose.position.y = correct_pose_static.pose.position.y

        rospy.loginfo("correct_pose.pose.position.x = " + str(correct_pose.pose.position.x))
        rospy.loginfo("correct_pose.pose.position.y = " + str(correct_pose.pose.position.y))
        rospy.loginfo("correct_pose.pose.position.z = " + str(correct_pose.pose.position.z))

        ptp_pre_goal = PtpGoal()
        ptp_pre_goal.pose.position.x = pre_grasp.pose.position.x #0.472 #pre_grasp.pose.position.x
        ptp_pre_goal.pose.position.y = pre_grasp.pose.position.y #-0.405 #pre_grasp.pose.position.y
        ptp_pre_goal.pose.position.z = pre_grasp.pose.position.z+0.1 #0.392 #pre_grasp.pose.position.z+0.2
        rospy.loginfo("ptp_pre_goal.pose.position.z = " + str(ptp_pre_goal.pose.position.z))
        #do not change values below
        ptp_pre_goal.pose.orientation.w = 0.111 #-0.393 #0.577 #0.287
        ptp_pre_goal.pose.orientation.x = 0.165 #0.472 #-0.710 #0.912
        ptp_pre_goal.pose.orientation.y = -0.583 #0.570 #-0.290 #0.290
        ptp_pre_goal.pose.orientation.z = 0.787 #-0.546 #0.281 #0.028

#	- Rotation: in Quaternion [-0.092, -0.662, 0.743, -0.035]
#       position: [0.0, 0.0, 0.0, 0.8537089540571047, 0.6674627751816875, 0.08086459490340127, 1.4226178733005779, -1.4423680191161456]
#	- Rotation: in Quaternion [0.472, 0.570, -0.546, -0.393]
#	[7.480065200124598e-05, 4.3229067433648816e-05, 0.00024683946092034914, 1.0496438046761931, 0.954026290671534, 0.07962052421257972, 1.0440559585430078, -0.23788139572981912]
#	-0.004, -0.578, 0.798, -0.169
#	- Rotation: in Quaternion [0.165, -0.583, 0.787, 0.111]
	
        ptp_goal = PtpGoal()
        ptp_goal.pose.position.x = correct_pose.pose.position.x #0.489 #correct_pose.pose.position.x
        ptp_goal.pose.position.y = correct_pose.pose.position.y #-0.416 #correct_pose.pose.position.y
        ptp_goal.pose.position.z = correct_pose.pose.position.z #0.180 #correct_pose.pose.position.z
        rospy.loginfo("ptp_goal.pose.position.z = " + str(ptp_goal.pose.position.z))
        #do not change values below
        ptp_goal.pose.orientation.w = 0.111 #-0.393 #0.613 #0.287
        ptp_goal.pose.orientation.x = 0.165 #0.472 #-0.682 #0.912
        ptp_goal.pose.orientation.y = -0.583 #0.570 #-0.300 #0.290
        ptp_goal.pose.orientation.z = 0.787 #-0.546 #0.265 #0.028
        
	#joint_ptp_goal = JointPtpGoal()
	#joint_ptp_goal.joints.data = [0.0, 0.0, 0.0, 1.05, 0.95, 0.008, 1.04, -0.24]

        ptp_retract_goal = PtpGoal()
        ptp_retract_goal.pose.position.x = 0.306
        ptp_retract_goal.pose.position.y = -0.480
        ptp_retract_goal.pose.position.z = 0.425
        rospy.loginfo("ptp_retract_goal.pose.position.z = " + str(ptp_retract_goal.pose.position.z))
        #do not change values below
        ptp_retract_goal.pose.orientation.w = -0.348
        ptp_retract_goal.pose.orientation.x = 0.862
        ptp_retract_goal.pose.orientation.y = 0.309
        ptp_retract_goal.pose.orientation.z = -0.200
        
        ptp_reset_goal = PtpGoal()
        ptp_reset_goal.pose.position.x = 0.315
        ptp_reset_goal.pose.position.y = -0.499
        ptp_reset_goal.pose.position.z = 0.575
        rospy.loginfo("ptp_reset_goal.pose.position.z = " + str(ptp_reset_goal.pose.position.z))
        #do not change values below
        ptp_reset_goal.pose.orientation.w = -0.071
        ptp_reset_goal.pose.orientation.x = 0.944
        ptp_reset_goal.pose.orientation.y = 0.296
        ptp_reset_goal.pose.orientation.z = -0.127
        
        open_hand = ActuateHandGoal()
        open_hand.command = 0
        open_hand.force_limit = 1
        close_hand = ActuateHandGoal()
        close_hand.command = 1
        close_hand.grasp_type = 0
        close_hand.force_limit = 1

        self._visualize_grasp(ptp_goal)

	print 'Ready to start planning the grasp...'
	ch = raw_input('Do you want to continue (y)?')
        if ch  != 'y':
            error = 'Aborted by user.'
            rospy.logerr(error)
            self.server.set_aborted(self.grasp_result, error)
            return


        # open hand
        rospy.loginfo("Opening hand...")
        self.metahand.send_goal(open_hand)
        self.metahand.wait_for_result()
        if self.metahand.get_state() == GoalStatus.ABORTED:
            error = 'Could not open hand.'
            rospy.logerr(error)
            #self.server.set_aborted(self.grasp_result, error)
            #return

	# pre approach goal
	joint_ptp_goal = JointPtpGoal()
        base_z = self.joint_state.position[2] + 0.75 # add 80 degree rotation to the pose
        if base_z > 3.14:
		base_z = 3.14
        rospy.loginfo('Adjusted z rotation from ' + str(self.joint_state.position[2]) + ' to ' + str(base_z))
	# [x,y,z, 0.941535318280861, 0.6804689687675491, -0.001017876019763093, -0.4048675172436286, -1.7988550094944915]
        joint_ptp_goal.joints.data = (self.joint_state.position[0], self.joint_state.position[1], base_z, 0.942, 0.680, -0.001, -0.405, -1.799)
        print joint_ptp_goal
        self.joint_ptp.send_goal(joint_ptp_goal)
        self.joint_ptp.wait_for_result()

        ##########################
	print '--- APPROACH GOAL ---'
        print ptp_pre_goal
        #ch = raw_input('Do you want to continue (y)?')
        #if ch  != 'y':
        #    error = 'Aborted by user.'
        #    rospy.logerr(error)
        #    self.server.set_aborted(self.grasp_result, error)
        #    return
        ##########################

        # move to pre pose
        rospy.loginfo("Approaching pre pose...")
        self.ptp.send_goal(ptp_pre_goal)
        self.ptp.wait_for_result()
        result = self.ptp.get_result()
        rospy.loginfo(result.result_status)
        if result.result_status == "execution failed.":
            error = 'Approaching pre pose failed.'
            rospy.logerr(error)
            #self.server.set_aborted(self.grasp_result, error)
            #return

        ##########################
        ch = raw_input('Do you want to continue (y)?')
        if ch  != 'y':
            error = 'Aborted by user.'
            rospy.logerr(error)
            self.server.set_aborted(self.grasp_result, error)
            return
        ##########################

        # move to grasp pose
        rospy.loginfo("Approaching grasp pose...")
	t = self.tf_listener.getLatestCommonTime("/origin", "/hand_base_link")
        current_position, current_orientation = self.tf_listener.lookupTransform("/origin", "/hand_base_link", t)
        h = current_position[2]
        ptp_goal.pose.position.x = current_position[0]
        ptp_goal.pose.position.y = current_position[1]
        h -= 0.20
        if h < 0.125:
            ptp_goal.pose.position.z = 0.125
        elif h > 0.20:
	    ptp_goal.pose.position.z = 0.20
	else:
            ptp_goal.pose.position.z = h
	
        rospy.loginfo("ptp_goal.pose.position.z = " + str(ptp_goal.pose.position.z))
        #do not change values below
        ptp_goal.pose.orientation.w = current_orientation[3]
        ptp_goal.pose.orientation.x = current_orientation[0]
        ptp_goal.pose.orientation.y = current_orientation[1]
        ptp_goal.pose.orientation.z = current_orientation[2]
	print '--- GRASP POSE ---'
        print ptp_goal
        self._visualize_grasp(ptp_goal)

        self.ptp.send_goal(ptp_goal)
        self.ptp.wait_for_result()
        result = self.ptp.get_result()
        #self.joint_ptp.send_goal(joint_ptp_goal)
        #self.joint_ptp.wait_for_result()
        #result = self.joint_ptp.get_result()
        rospy.loginfo(result.result_status)
        if result.result_status == "execution failed.":
            error = 'Approaching grasp pose failed.'
            rospy.logerr(error)
            #self.server.set_aborted(self.grasp_result, error)
            #return
            
        ##########################
        print ptp_goal
        #ch = raw_input('Do you want to continue (y)?')
        #if ch  != 'y':
        #    error = 'Aborted by user.'
        #    rospy.logerr(error)
        #    self.server.set_aborted(self.grasp_result, error)
        #    return
        ##########################

        # grasp the object
	sleep(1.0)
        rospy.loginfo("Grasping...")
        self.metahand.send_goal(close_hand)
        result = self.metahand.wait_for_result()
        if self.metahand.get_state() == GoalStatus.ABORTED:
            error = 'Grasping failed.'
            rospy.logerr(error)
            #self.server.set_aborted(self.grasp_result, error)
            #return

        ##########################
        #ch = raw_input('Do you want to continue (y)?')
        #if ch  != 'y':
        #    error = 'Aborted by user.'
        #    rospy.logerr(error)
        #    self.server.set_aborted(self.grasp_result, error)
        #    return
        ##########################

        # retract the arm
        ptp_goal.pose.position.z+=0.2
	joint_ptp_goal = JointPtpGoal()
        joint_ptp_goal.joints.data = [self.joint_state.position[0], self.joint_state.position[1], self.joint_state.position[2], 0.7, 1.6, 0.00, -1.7, -1.8]
        rospy.loginfo("Retracting...")
	#print ptp_goal
        #self.ptp.send_goal(ptp_goal)
        #self.ptp.wait_for_result()
        #result = self.ptp.get_result()
	print joint_ptp_goal
	self.joint_ptp.send_goal(joint_ptp_goal)
	self.joint_ptp.wait_for_result()
	print 'Finished retracting now returning'
	#self.server.set_succeeded(self.grasp_result, success)
	#return
	#result = self.joint_ptp.get_result()
        #rospy.loginfo(result.result_status)
        #if result.result_status == "execution failed.":
            #error = 'Retracting the arm failed.'
            #rospy.logerr(error)
            # Try retract to the approach pose
            #rospy.loginfo("Retracting to approach pose...")
            #self.ptp.send_goal(ptp_pre_goal)
            #self.ptp.wait_for_result()
            #result = self.ptp.get_result()
            #rospy.loginfo(result.result_status)
            #if result.result_status == "execution failed.":
            #    error = 'Retracting the arm failed again.'
            #    rospy.logerr(error)
            #    self.server.set_aborted(self.grasp_result, error)
            #    return

        # we're done
        success = 'success'
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
        graspMarker.scale.x = 0.15
        graspMarker.scale.y = 0.15
        graspMarker.scale.z = 0.15
        graspMarker.color.r = 0.0
        graspMarker.color.g = 1.0
        graspMarker.color.b = 0.0
        graspMarker.color.a = 1.0
        graspMarker.lifetime = rospy.Duration(secs=30)
        self.markerPub.publish(graspMarker)


    def _visualize_poses(self, goal):
        poses = [goal.heap_center_pose, goal.heap_center_pose_static]
        which = ["computed", "static"]
        colors = [(1,0,0), (0,0,1)]

        for p, w, c in zip(poses, which, colors):
            correct_pose = None
            if not p.header.frame_id == 'origin':
                p.header.stamp = self.tf_listener.getLatestCommonTime(p.header.frame_id, 'origin')
                correct_pose = self.tf_listener.transformPose('origin', p)                                
            else:
                correct_pose = copy(p)
                   
            poseMarker = Marker()
            poseMarker.header.frame_id = "/origin"
            poseMarker.header.stamp = rospy.get_rostime()
            poseMarker.ns = w
            poseMarker.id = 0
            poseMarker.type = 2
            poseMarker.action = 0
            poseMarker.pose.position = correct_pose.pose.position
            poseMarker.pose.orientation.x = 0
            poseMarker.pose.orientation.y = 0
            poseMarker.pose.orientation.z = 0
            poseMarker.pose.orientation.w = 1.0
            poseMarker.scale.x = 0.2
            poseMarker.scale.y = 0.2
            poseMarker.scale.z = 0.2
            poseMarker.color.r = c[0]
            poseMarker.color.g = c[1]
            poseMarker.color.b = c[2]
            poseMarker.color.a = 1.0
            poseMarker.lifetime = rospy.Duration(secs=30)
            self.markerPub.publish(poseMarker)
