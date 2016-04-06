import rospy
import actionlib
import numpy as np
import tf
import sys
from geometry_msgs.msg import PoseStamped

from squirrel_manipulation_msgs.msg import BlindGraspAction
from squirrel_manipulation_msgs.msg import BlindGraspResult
from squirrel_manipulation_msgs.msg import BlindGraspFeedback

from kclhand_control.srv import graspCurrent, graspPreparation

from moveit_commander import roscpp_initialize, roscpp_shutdown, MoveGroupCommander



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
#        self._group.set_planner_id('RRTConnectkConfigDefault')
        self._group.set_num_planning_attempts(5)
        self._server.start()
        self._result = BlindGraspResult()
        self._feedback = BlindGraspFeedback()
        self._span = .1
        self._dist_2_hand = .18


    
    def execute_grasp(self, goal):

        self._feedback.current_phase = 'BlindGrasp: parsing action goal'
        self._feedback.current_status = 'BlindGrasp: preparing action execution'
        self._feedback.percent_completed = 0.0

        self._server.publish_feedback(self._feedback)
        
        rospy.loginfo(rospy.get_caller_id() + ': Requested blind grasp of object %s at pose %s'.format(goal.object_id, goal.heap_center_pose))

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

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = goal.heap_center_pose.header.frame_id
        pose.pose.position.x = goal.heap_center_pose.pose.position.x
        pose.pose.position.x = goal.heap_center_pose.pose.position.y
        pose.pose.position.x = goal.heap_center_pose.pose.position.z + d + self._dist_2_hand
        pose.pose.orientation.w = 0.0
        pose.pose.orientation.x = -1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        rospy.loginfo(rospy.get_caller_id() + ': Computed gripper pose: %s'.format(pose))

        '''
        if goal.heap_bounding_box.x > self._span:
            d_ = goal.heap_bounding_cylinder.diameter/2.0
            pose.pose.position.x = pose.pose.position.x + d_
        '''

        self._feedback.current_phase = 'BlindGrasp: moving gripper'
        self._feedback.current_status = 'BlindGrasp: executing action'
        self._feedback.percent_completed = 0.6
        self._server.publish_feedback(self._feedback)
        
        self._group.clear_pose_targets()
        self._group.set_start_state_to_current_state()
        self._group.set_pose_target(pose)
        plan = self._group.plan()

        if self.is_empty(plan):
            self._feedback.current_phase = 'BlindGrasp: aborted - no plan found'
            self._feedback.current_status = 'BlindGrasp: aborted action execution'
            self._feedback.percent_completed = 1
            self._server.publish_feedback(self._feedback)

            self._result.result_status = 'BlindGrasp: failed to grasp object %s'.format(goal.object_id) 
            rospy.loginfo('BlindGrasp: failed - no motion plan found')
            self._server.set_succeeded(self._result)
           
        else:
            self._feedback.current_phase = 'BlindGrasp: grasping'
            self._feedback.current_status = 'BlindGrasp: executing action'
            self._feedback.percent_completed = 9
            self._server.publish_feedback(self._feedback)

            self._prepareGrasp()
            self._group.go(wait=True)
            self._closeFinger(1.0)

            # lift object
            self._group.clear_pose_targets()
            self._group.set_start_state_to_current_state()
            self._pose.pose.position.z = pose.pose.position.z + 0.3
            self._group.set_pose_target(pose)
            self._group.plan()
            self._group.go(wait=True)
            
            self._feedback.current_phase = 'BlindGrasp: grasping succeeded'
            self._feedback.current_status = 'BlindGrasp: finished action execution'
            self._feedback.percent_completed = 1
            self._server.publish_feedback(self._feedback)

            self._result.result_status = 'BlindGrasp: grasped object %s'.format(goal.object_id) 
            rospy.loginfo('BlindGrasp: succeeded')
            self._server.set_succeeded(self._result)            
            

    def is_empty(self, plan):
        if len(plan.joint_trajectory.points) == 0 and \
            len(plan.multi_dof_joint_trajectory.points) == 0:
            return True
        return False
