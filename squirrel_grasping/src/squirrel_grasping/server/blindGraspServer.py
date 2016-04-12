import rospy
import actionlib
import sys
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool

from tf.transformations import quaternion_from_euler

from squirrel_manipulation_msgs.msg import BlindGraspAction
from squirrel_manipulation_msgs.msg import BlindGraspResult
from squirrel_manipulation_msgs.msg import BlindGraspFeedback

from kclhand_control.srv import graspCurrent, graspPreparation

from moveit_commander import roscpp_initialize, roscpp_shutdown, MoveGroupCommander, PlanningSceneInterface


class BlindGraspServer(object):


    def __init__(self):
        while rospy.get_time() == 0.0: pass
        rospy.loginfo(rospy.get_caller_id() + ': starting BlindGraspServer')
        self._server = actionlib.SimpleActionServer('blindGrasp', BlindGraspAction, execute_cb=self.execute_grasp, auto_start=False)
        self._prepareGrasp = rospy.ServiceProxy('hand_controller/prepareGrasp', graspPreparation)
        self._closeFinger = rospy.ServiceProxy('hand_controller/closeFinger', graspCurrent)
        self._openFinger = rospy.ServiceProxy('hand_controller/openFinger', graspPreparation)
        self._rotatory_lock = rospy.Publisher('/twist_mux/locks/pause_rotatory', Bool, queue_size=10)
        roscpp_initialize(sys.argv)
        self._group = MoveGroupCommander('arm')
        self._result = BlindGraspResult()
        self._feedback = BlindGraspFeedback()
        self._dist_2_hand = .2
        self._server.start()

    
    def execute_grasp(self, goal):
        
        rospy.loginfo(rospy.get_caller_id() + ': BlindGrasp called with goal\n{}'.format(goal))
        
        self._rotatory_lock.publish(False)

        if self._server.is_preempt_requested():
            rospy.loginfo('BlindGrasp: preempted')
            self._server.set_preempted()
            return

        pre_pose = PoseStamped()
        pre_pose.header.stamp = rospy.Time.now()
        pre_pose.header.frame_id = 'odom'#goal.heap_center_pose.header.frame_id
        pre_pose.pose.position.x = 0.392694652081#goal.heap_center_pose.pose.position.x
        pre_pose.pose.position.y = 0.135437235236#goal.heap_center_pose.pose.position.y
        pre_pose.pose.position.z = 0.481809170246#goal.heap_center_pose.pose.position.z + d + self._dist_2_hand + 0.15
        pre_pose.pose.orientation.x = -0.0416706353426
        pre_pose.pose.orientation.y = 0.999013662338
        pre_pose.pose.orientation.z = -0.0152916312218
        pre_pose.pose.orientation.w = -0.00118487002328
        #grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y, grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w = \
        #  quaternion_from_euler( 3.142, 0.050, 2.094 )

        grasp_pose = PoseStamped()
        grasp_pose.header.stamp = rospy.Time.now()
        grasp_pose.header.frame_id = 'odom'#goal.heap_center_pose.header.frame_id
        grasp_pose.pose.position.x = 0.394616365433#goal.heap_center_pose.pose.position.x
        grasp_pose.pose.position.y = 0.124969959259#goal.heap_center_pose.pose.position.y
        grasp_pose.pose.position.z = 0.411426967382#goal.heap_center_pose.pose.position.z + d + self._dist_2_hand
        grasp_pose.pose.orientation.x = -0.0458614192903
        grasp_pose.pose.orientation.y = 0.998826861382
        grasp_pose.pose.orientation.z = -0.0154969207942
        grasp_pose.pose.orientation.w = -0.00127845082898        
        #grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y, grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w = \
        #  quaternion_from_euler( 3.142, 0.050, 2.094 )

        rospy.loginfo(rospy.get_caller_id() + ': Computed pre grasp pose:\n{}'.format(pre_pose))
        rospy.loginfo(rospy.get_caller_id() + ': Computed grasp pose:\n{}'.format(grasp_pose))

        self._group.set_planner_id('RRTConnectkConfigDefault')
        self._group.set_num_planning_attempts(1)
        self._group.set_planning_time(5.0)        
        self._group.clear_pose_targets()
        self._group.set_start_state_to_current_state()
        self._group.set_pose_reference_frame("odom")
        pre_pose.header.stamp = rospy.Time.now()
        self._group.set_pose_target(pre_pose)
        plan = self._group.plan()

        if self._is_empty(plan):
            rospy.logerror('BlindGrasp: failed - no motion plan found for pre grasp pose')
            self._rotatory_lock.publish(True)            
        else:
            self._group.go(wait=True)
            self._group.clear_pose_targets()
            self._group.set_start_state_to_current_state()
            self._group.set_pose_reference_frame("odom")
            grasp_pose.header.stamp = rospy.Time.now()
            self._group.set_pose_target(grasp_pose)
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
                self._group.set_pose_reference_frame("odom")
                pre_pose.header.stamp = rospy.Time.now()
                self._group.set_pose_target(pre_pose)
                plan = self._group.plan()

                if self._is_empty(plan):
                    rospy.logerror('BlindGrasp: retraction failed - no motion plan found')
                else:
                    self._group.go(wait=True)
                    rospy.loginfo('BlindGrasp: succeeded')
                    self._result.result_status = 'BlindGrasp: succeeded' 
                    self._server.set_succeeded(self._result) 
                    self._rotatory_locak.publish(True)                    
            

    def _is_empty(self, plan):
        if len(plan.joint_trajectory.points) == 0 and \
            len(plan.multi_dof_joint_trajectory.points) == 0:
            return True
        return False
