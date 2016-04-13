import rospy
import actionlib
import sys
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool

from tf.transformations import quaternion_from_euler
from tf import TransformListener

from squirrel_manipulation_msgs.msg import BlindGraspAction
from squirrel_manipulation_msgs.msg import BlindGraspResult
from squirrel_manipulation_msgs.msg import BlindGraspFeedback

from kclhand_control.srv import graspCurrent, graspPreparation

from moveit_commander import roscpp_initialize, roscpp_shutdown, MoveGroupCommander, PlanningSceneInterface


class BlindGraspServer(object):


    def __init__(self):
        while rospy.get_time() == 0.0: pass
        rospy.loginfo(rospy.get_caller_id() + ': starting BlindGraspServer')

        rospy.wait_for_service("/get_planning_scene")
        rospy.sleep(5.0)

        self._server = actionlib.SimpleActionServer('blindGrasp', BlindGraspAction, execute_cb=self._execute_grasp, auto_start=False)
        self._prepareGrasp = rospy.ServiceProxy('hand_controller/prepareGrasp', graspPreparation)
        self._closeFinger = rospy.ServiceProxy('hand_controller/closeFinger', graspCurrent)
        self._openFinger = rospy.ServiceProxy('hand_controller/openFinger', graspPreparation)
        self._rotatory_lock = rospy.Publisher('/twist_mux/locks/pause_rotatory', Bool, queue_size=10)
        self._transformer = TransformListener()

        roscpp_initialize(sys.argv)
        self._group = MoveGroupCommander('arm')
        self._result = BlindGraspResult()
        self._feedback = BlindGraspFeedback()
        self._dist_2_hand = .25

        self._retract_pose = PoseStamped()
        self._retract_pose.header.frame_id = 'base_link'
        self._retract_pose.header.stamp = rospy.Time.now()
        self._retract_pose.pose.position.x = 0.114
        self._retract_pose.pose.position.y = -0.205
        self._retract_pose.pose.position.z = 0.653
        self._retract_pose.pose.orientation.x = 0.912
        self._retract_pose.pose.orientation.y = 0.292
        self._retract_pose.pose.orientation.z = 0.028
        self._retract_pose.pose.orientation.w = 0.287

        self._server.start()

    
    def _execute_grasp(self, goal):
        
        rospy.loginfo(rospy.get_caller_id() + ': BlindGrasp called')
        
        self._rotatory_lock.publish(False)

        if self._server.is_preempt_requested():
            rospy.loginfo('BlindGrasp: preempted')
            self._server.set_preempted()
            return

        pre_pose = PoseStamped()
        pre_pose.header.stamp = rospy.Time.now()
        pre_pose.header.frame_id = goal.heap_center_pose.header.frame_id
        pre_pose.pose.position.x = goal.heap_center_pose.pose.position.x
        pre_pose.pose.position.y = goal.heap_center_pose.pose.position.y
        pre_pose.pose.position.z = 0.481809170246
        pre_pose.pose.orientation.x = -0.0416706353426
        pre_pose.pose.orientation.y = 0.999013662338
        pre_pose.pose.orientation.z = -0.0152916312218
        pre_pose.pose.orientation.w = -0.00118487002328

        d = goal.heap_bounding_cylinder.height/2.0

        grasp_pose = PoseStamped()
        grasp_pose.header.stamp = rospy.Time.now()
        grasp_pose.header.frame_id = goal.heap_center_pose.header.frame_id
        grasp_pose.pose.position.x = goal.heap_center_pose.pose.position.x
        grasp_pose.pose.position.y = goal.heap_center_pose.pose.position.y
        grasp_pose.pose.position.z = goal.heap_center_pose.pose.position.z + d + self._dist_2_hand
        grasp_pose.pose.orientation.x = -0.0458614192903
        grasp_pose.pose.orientation.y = 0.998826861382
        grasp_pose.pose.orientation.z = -0.0154969207942
        grasp_pose.pose.orientation.w = -0.00127845082898        

        rospy.loginfo(rospy.get_caller_id() + ': Computed pre grasp pose:\n{}'.format(pre_pose))
        rospy.loginfo(rospy.get_caller_id() + ': Computed grasp pose:\n{}'.format(grasp_pose))

        self._group.set_planner_id('RRTConnectkConfigDefault')
        self._group.set_num_planning_attempts(1)
        self._group.set_planning_time(5.0)        
        self._group.clear_pose_targets()
        self._group.set_start_state_to_current_state()
        self._group.set_pose_reference_frame('odom')
        pre_pose.header.stamp = self._transformer.getLatestCommonTime('odom', pre_pose.header.frame_id)
        pre_pose_tfed = self._transformer.transformPose('odom', pre_pose)
        self._group.set_pose_target(pre_pose_tfed)
        plan = self._group.plan()

        if self._is_empty(plan):
            rospy.logerror('BlindGrasp: failed - no motion plan found for pre grasp pose')
            self._rotatory_lock.publish(True)            
        else:
            self._group.go(wait=True)
            self._group.clear_pose_targets()
            self._group.set_start_state_to_current_state()
            self._group.set_pose_reference_frame('odom')
            grasp_pose.header.stamp = self._transformer.getLatestCommonTime('odom', grasp_pose.header.frame_id)
            grasp_pose_tfed = self._transformer.transformPose('odom', grasp_pose)
            self._group.set_pose_target(grasp_pose_tfed)
            plan = self._group.plan()
            
            if self._is_empty(plan):
                rospy.logerror('BlindGrasp: failed - no motion plan found for grasp pose')
                self._rotatory_lock.publish(True) 
            else:
                self._prepareGrasp()
                self._group.go(wait=True)
                self._closeFinger(1.0)
                self._group.clear_pose_targets()
                self._group.set_start_state_to_current_state()
                self._group.set_pose_reference_frame('odom')
                self._retract_pose.header.stamp = self._transformer.getLatestCommonTime('odom', self._retract_pose.header.frame_id)
                retract_pose_tfed = self._transformer.transformPose('odom', self._retract_pose)
                self._group.set_pose_target(retract_pose_tfed)
                plan = self._group.plan()

                if self._is_empty(plan):
                    rospy.logerror('BlindGrasp: retraction failed - no motion plan found')
                    self._rotatory_locak.publish(True)                    
                else:
                    self._group.go(wait=True)
                    rospy.loginfo('BlindGrasp: succeeded')
                    self._result.result_status = 'BlindGrasp: succeeded' 
                    self._server.set_succeeded(self._result) 
                    self._rotatory_lock.publish(True)                    
            

    def _is_empty(self, plan):
        if len(plan.joint_trajectory.points) == 0 and \
            len(plan.multi_dof_joint_trajectory.points) == 0:
            return True
        return False
