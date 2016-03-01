import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from squirrel_manipulation_msgs.msg import Grasp
from squirrel_manipulation_msgs.srv.GraspImportResponse
from geometry_msgs import PoseStamped


def store_grasp(object_id, ground_truth, learned_object_pose, gripper_config, gripper_pose, quality):
    msg_store = MessageStoreProxy()

    g = Grasp();
    g.object_id = object_id
    g.ground_truth = ground_truth
    g.quality = quality
    g.gripper_config = gripper_config
    g.learned_object_pose = learned_object_pose
    g.gripper_pose = gripper_pose

    try:
        p_id = msg_store.insert(g)
        return p_id
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' % e)
        return None


def grasp_importer(msg):
#    store_grasp(msg.object_id, )
    ground_truth_pose = 0 #get from knowledge base - this must not be done, we can also get it everytime we do a grasp
    grasps = 0 #read in grasps
    for g in grasps:
        pose = PoseStamped()
        pose.header.seq = 1 # what to ideally out here?
        pose.header.stamp = rospy.rostime.now()
        pose.header.frame_id = '/world'
        pose.pose.position.x = g[0]
        pose.pose.position.x = g[1]
        pose.pose.position.x = g[2]
        pose.pose.orientation.w = g[3]
        pose.pose.orientation.w = g[4]
        pose.pose.orientation.w = g[5]
        pose.pose.orientation.w = g[6]
        store_grasp(msg.object_id, ground_truth_pose, msg.learned_object_pose, msg.gripper_config, msg.gripper_pose, g[7])    

    return GraspImportResponse(True)
