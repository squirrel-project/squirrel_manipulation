import rospy
import actionlib
import numpy as np
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from tf.transformations import quaternion_matrix
from ..db.util import grasp_import_callback


from squirrel_manipulation_msgs.srv import GraspImport
from squirrel_manipulation_msgs.msg import Grasp
from squirrel_manipulation_msgs.msg import GraspActionAction
from squirrel_manipulation_msgs.msg import GraspActionResult
from squirrel_manipulation_msgs.msg import GraspActionFeedback
from squirrel_object_perception_msgs.msg import ObjectToDB


class GraspServer(object):

    _result = GraspActionResult()
    _feedback = GraspActionFeedback()

    def __init__(self):
        while rospy.get_time() == 0.0: pass
        rospy.loginfo(rospy.get_caller_id() + ': starting GraspServer')
        self._server = actionlib.SimpleActionServer('grasp', GraspActionAction, execute_cb=self.execute_grasp, auto_start=False)
        self._importer = rospy.Service('grasp_importer', GraspImport, grasp_import_callback)
        self._server.start()

    
    def execute_grasp(self, goal):

        self._feedback.current_phase = 'parsing action goal'
        self._feedback.current_status = 'preparing action execution'
        self._feedback.percent_completed = 0.0

        self._server.publish_feedback(self._feedback)
        
        rospy.loginfo(rospy.get_caller_id() + ': Requested grasping of object %s with gripper config %s' %
                      (goal.grasp.objbect_id, goal.grasp.gripper_config))

        # check that preempt has not been requested by the client
        # this needs refinement for the next iteration after Y2 review
        if self._server.is_preempt_requested():
            rospy.loginfo('GraspAction: preempted')
            self._server.set_preempted()
            return

        # map learned object pose to current object pose
        self._feedback.current_phase = 'computing gripper transform'
        self._feedback.current_status = 'preparing action execution'
        self._feedback.percent_completed = 0.2
        self._server.publish_feedback(self._feedback)

        transformation = self.__inverseTimes(goal.grasp.learned_object_pose.pose, goal.object_pose.pose)

        if transformation == None:
            self._result.result_status = 'failed to grasp object %s'.format(goal.grasp.object_id) 
            rospy.loginfo('GraspAction: failed')
            self.server.set_succeeded(self._result)
            return            

        # TODO: compute new gripper pose given goal.gripper_pose

        self._feedback.current_phase = 'configuring gripper'
        self._feedback.current_status = 'preparing action execution'
        self._feedback.percent_completed = 0.5
        self._server.publish_feedback(self._feedback)
        # TODO: read gripper config and set
        current_config = None

        self._feedback.current_phase = 'moving gripper'
        self._feedback.current_status = 'executing action'
        self._feedback.percent_completed = 0.75
        self._server.publish_feedback(self._feedback)
        # TODO: move arm using squirrel arm control - check with SH

        self._feedback.current_phase = 'grasping object'
        self._feedback.current_status = 'executing action'
        self._feedback.percent_completed = 1
        self._server.publish_feedback(self._feedback)
        # TODO: grasp object

        self._result.result_status = 'object %s grasped'.format(goal.grasp.object_id) 
        rospy.loginfo('GraspAction: succeeded')

        self.server.set_succeeded(self._result)


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
