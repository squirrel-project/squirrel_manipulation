import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from squirrel_manipulation_msgs.msg import Grasp


def store_grasp(object_id, ground_truth, gripper_config, learned_object_pose, gripper_pose):
    msg_store = MessageStoreProxy()

    g = Grasp();
    g.objectID = object_id
    g.groundTruth = ground_truth
    g.gripperConfiguration = gripper_config
    g.learnedObjectPose = learned_object_pose
    g.gripperPose = gripper_pose

    try:
        p_id = msg_store.insert(g)
        return p_id
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' % e)
        return None
