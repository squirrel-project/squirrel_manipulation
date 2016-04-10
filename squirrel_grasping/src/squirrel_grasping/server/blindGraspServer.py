import rospy
import actionlib
import sys
from geometry_msgs.msg import Pose, PoseStamped

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
        roscpp_initialize(sys.argv)
        self._group = MoveGroupCommander('arm')
        self._result = BlindGraspResult()
        self._feedback = BlindGraspFeedback()
        self._dist_2_hand = .30
        self._server.start()

    
    def execute_grasp(self, goal):
        self._feedback.current_phase = 'BlindGrasp: parsing action goal'
        self._feedback.current_status = 'BlindGrasp: preparing action execution'
        self._feedback.percent_completed = 0.0

        self._server.publish_feedback(self._feedback)
        
        rospy.loginfo(rospy.get_caller_id() + ': Requested blind grasp of object {} at pose\n{}'.format(goal.object_id, goal.heap_center_pose))

        # check that preempt has not been requested by the client
        # this needs refinement for the next iteration after Y2 review
        if self._server.is_preempt_requested():
            rospy.loginfo('BlindGrasp: preempted')
            self._server.set_preempted()
            return

        self._feedback.current_phase = 'BlindGrasp: computing gripper pose'
        self._feedback.current_status = 'BlindGrasp: preparing action execution'
        self._feedback.percent_completed = 0.3
        self._server.publish_feedback(self._feedback)

        d = goal.heap_bounding_cylinder.height/2.0

        '''
        pre_pose = PoseStamped()
        pre_pose.header.stamp = rospy.Time.now()
        pre_pose.header.frame_id = goal.heap_center_pose.header.frame_id
        pre_pose.pose.position.x = goal.heap_center_pose.pose.position.x
        pre_pose.pose.position.y = goal.heap_center_pose.pose.position.y
        pre_pose.pose.position.z = goal.heap_center_pose.pose.position.z + d + self._dist_2_hand + 0.2
        pre_pose.pose.orientation.x, pre_pose.pose.orientation.y, pre_pose.pose.orientation.z, pre_pose.pose.orientation.w = \
          quaternion_from_euler( 3.142, 0.050, 2.094 )
        '''

        pre_pose = Pose()
        pre_pose.position.x = goal.heap_center_pose.pose.position.x
        pre_pose.position.y = goal.heap_center_pose.pose.position.y
        pre_pose.position.z = goal.heap_center_pose.pose.position.z + d + self._dist_2_hand + 0.2
        pre_pose.orientation.x, pre_pose.orientation.y, pre_pose.orientation.z, pre_pose.pose.orientation.w = \
          quaternion_from_euler 3.104, -0.116, 1.916)

        rospy.loginfo(rospy.get_caller_id() + ': Computed pre gripper pose:\n{}'.format(pre_pose))

        '''
        grasp_pose = PoseStamped()
        grasp_pose.header.stamp = rospy.Time.now()
        grasp_pose.header.frame_id = goal.heap_center_pose.header.frame_id
        grasp_pose.pose.position.x = goal.heap_center_pose.pose.position.x
        grasp_pose.pose.position.y = goal.heap_center_pose.pose.position.y
        grasp_pose.pose.position.z = goal.heap_center_pose.pose.position.z + d + self._dist_2_hand
        grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y, grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w = \
          quaternion_from_euler( 3.142, 0.050, 2.094 )
        '''
        
        grasp_pose = Pose()
        grasp_pose.position.x = goal.heap_center_pose.pose.position.x
        grasp_pose.position.y = goal.heap_center_pose.pose.position.y
        grasp_pose.position.z = goal.heap_center_pose.pose.position.z + d + self._dist_2_hand
        grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w = \
          quaternion_from_euler(3.104, -0.116, 1.916)

        rospy.loginfo(rospy.get_caller_id() + ': Computed grasp gripper pose:\n{}'.format(grasp_pose))

        self._feedback.current_phase = 'BlindGrasp: moving gripper'
        self._feedback.current_status = 'BlindGrasp: executing action'
        self._feedback.percent_completed = 0.4
        self._server.publish_feedback(self._feedback)

        self._group.set_planner_id('RRTConnectkConfigDefault')
        self._group.set_num_planning_attempts(1)
        self._group.set_planning_time(25.0)        
        self._group.clear_pose_targets()
        self._group.set_start_state_to_current_state()
        self._group.set_pose_target(pre_pose)
        plan = self._group.plan()

        if self.is_empty(plan):
            self._feedback.current_phase = 'BlindGrasp: aborted - no plan found'
            self._feedback.current_status = 'BlindGrasp: aborted action execution'
            self._feedback.percent_completed = 1
            self._server.publish_feedback(self._feedback)

            self._result.result_status = 'BlindGrasp: failed to grasp object {}'.format(goal.object_id) 
            rospy.loginfo('BlindGrasp: failed - no motion plan found')
            self._server.set_succeeded(self._result)
           
        else:
            self._feedback.current_phase = 'BlindGrasp: moving gripper to pre pose\n{}'.format(pre_pose) 
            self._feedback.current_status = 'BlindGrasp: preparing grasp'
            self._feedback.percent_completed = 0.6 
            self._server.publish_feedback(self._feedback)
            self._group.go(wait=True)
            self._group.clear_pose_targets()
            self._group.set_start_state_to_current_state()
            self._group.set_pose_target(grasp_pose)
            plan = self._group.plan()

            if self.is_empty(plan):
                self._feedback.current_phase = 'BlindGrasp: aborted - no plan found'
                self._feedback.current_status = 'BlindGrasp: aborted action execution'
                self._feedback.percent_completed = 1
                self._server.publish_feedback(self._feedback)

                self._result.result_status = 'BlindGrasp: failed to grasp object {}'.format(goal.object_id) 
                rospy.loginfo('BlindGrasp: failed - no motion plan found')
                self._server.set_succeeded(self._result)
            else:
                self._feedback.current_phase = 'BlindGrasp: attempting to grasp'
                self._feedback.current_status = 'BlindGrasp: moving ro grasp pose'
                self._feedback.percent_completed = 0.8
                self._server.publish_feedback(self._feedback)

                self._prepareGrasp()
                self._group.go(wait=True)
                self._closeFinger(1.0)

                self._group.clear_pose_targets()
                self._group.set_start_state_to_current_state()
                self._group.set_pose_target(pre_pose)
                plan = self._group.plan()

                if self.is_empty(plan):
                    self._feedback.current_phase = 'BlindGrasp: aborted - no plan found'
                    self._feedback.current_status = 'BlindGrasp: aborted action execution'
                    self._feedback.percent_completed = 1
                    self._server.publish_feedback(self._feedback)

                    self._result.result_status = 'BlindGrasp: failed to grasp object {}'.format(goal.object_id) 
                    rospy.loginfo('BlindGrasp: failed - no motion plan found')
                    self._server.set_succeeded(self._result)
                else:
                    self._group.go(wait=True)
                    self._feedback.current_phase = 'BlindGrasp: grasping succeeded'
                    self._feedback.current_status = 'BlindGrasp: finished action execution'
                    self._feedback.percent_completed = 1
                    self._server.publish_feedback(self._feedback)

                    self._result.result_status = 'BlindGrasp: grasped object {}'.format(goal.object_id) 
                    rospy.loginfo('BlindGrasp: succeeded')
                    self._server.set_succeeded(self._result)            
            

    def is_empty(self, plan):
        if len(plan.joint_trajectory.points) == 0 and \
            len(plan.multi_dof_joint_trajectory.points) == 0:
            return True
        return False
