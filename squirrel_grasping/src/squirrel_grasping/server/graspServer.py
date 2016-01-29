import rospy
import actionlib
import numpy as np

import squirrel_manipulation_msgs.msg.Grasp as Grasp
import squirrel_manipulation_msgs.msg.GraspAction as GraspAction
import squirrel_manipulation_msgs.msg.GraspActionResult as GraspActionResult
import squirrel_manipulation_msgs.msg.GraspActionFeedback as GraspActionFeedback

from gemoetry_msgs import Pose
from tf.transformations import quaternion_matrix

class GraspServer(object):

    _result = GraspActionResult()
    _feedback = GraspActionFeedback()

    def __init__(self):
        while rospy.get_time() == 0.0: pass
        rospy.loginfo(rospy.get_caller_id() + ': starting GraspServer')
        self._server = actionlib.SimpleActionServer('grasp', GraspAction, execute_cb=self.execute, auto_start=False)
        self._server.
        self._server.start()

    
    def execute(self, goal):

        self._feedback.current_phase = 'parsing action goal'
        self._feedback.current_status = 'started action execution'
        self._feedback.percent_completed = 0.0

        self._server.publish_feedback(self._feedback)
        
        rospy.loginfo(rospy.get_caller_id() + ': Requested grasping of object %s with gripper config %s and pose [pos, rot] [[%i, %i, %i], [%i, %i, %i, %i]]' %
                      (goal.grasp.objbectID, goal.grasp.gripperConfiguration, 
                      goal.grasp.gripperPose.position.x, goal.grasp.gripperPose.position.y, goal.grasp.gripperPose.position.z,
                      goal.grasp.gripperPose.orientation.w, goal.grasp.gripperPose.orientation.x, goal.grasp.gripperPose.orientation.y, goal.grasp.gripperPose.orientation.z))

        # check that preempt has not been requested by the client
        # this needs refinement for the next iteration after Y2 review
        if self._as.is_preempt_requested():
            rospy.loginfo('GraspAction: preempted')
            self._as.set_preempted()
            return

        # map learned object pose to current object pose
        self._feedback.current_phase = 'computing gripper transform'
        self._feedback.current_status = 'preparing action exection'
        self._feedback.percent_completed = 0.2
        self._server.publish_feedback(self._feedback)

        transformation = __computeGraspTransformation(goal.grasp, grasp.objectPose)


        self._feedback.current_phase = 'moving gripper to requested pose'
        self._feedback.current_status = 'moving arm'
        self._feedback.percent_completed = 0.4
        self._server.publish_feedback(self._feedback)
        # move arm using squirrel arm control

        self._feedback.current_phase = 'configuring gripper'
        self._feedback.current_status = 'reading gripper configuration'
        self._feedback.percent_completed = 0.6
        self._server.publish_feedback(self._feedback)
        # read gripper config
        current_config = None

        if current_config == goal.grasp.gripperConfiguration:
            self._feedback.current_phase = 'configuring gripper'
            self._feedback.current_status = 'setting gripper to homing position'
        self._feedback.percent_completed = 0.8
            self._server.publish_feedback(self._feedback)
            # move gripper to homing in current configuration
        else:
            self._feedback.current_phase = 'configuring gripper'
            self._feedback.current_status = 'setting gripper configuration'
            self._server.publish_feedback(self._feedback)
        self._feedback.percent_completed = 0.8
            # move gripper to new configuration

        self._feedback.current_phase = 'grasping object'
        self._feedback.current_status = 'closing gripper'
        self._feedback.percent_completed = 1
        self._server.publish_feedback(self._feedback)
        #grasp object

        self._result.result_status = 'object %s grasped'.format(goal.grasp.objectID) 
        rospy.loginfo('GraspAction: succeeded')

        self.server.set_succeeded(self._result)


    def __computeGraspTransformation(grasp, objectPose):
        assert isinstance(grasp, Grasp)

        gTP = grasp.groundTruth
        lOP = grasp.learnedObjectPose

        trans1 = self.__inverseTimes(lOP, gTP)
        trans2 = self.__inverseTimes(gTP, objectPose)

        # combine transformations and return


    def __inverseTimes(self, pose1, pose2):
        v1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
        m1 = quaternion_matrix([pose1.orientation.w,
                                pose1.orientation.x,
                                pose1.orientation.y,
                                pose1.orientation.z,])

        v2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
        m2 = quaternion_matrix([pose2.orientation.w,
                                pose2.orientation.x,
                                pose2.orientation.y,
                                pose2.orientation.z,])

        trans = (v2 - v1)*m1
        rot = m1.T()*m2

        return (rot, trans)
