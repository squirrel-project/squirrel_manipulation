import rospy
from numpy import fromstring
from mongodb_store.message_store import MessageStoreProxy
from squirrel_manipulation_msgs.msg import Grasp
from squirrel_manipulation_msgs.srv import GraspImportResponse
from geometry_msgs.msg import PoseStamped


def store_grasp(object_id, learned_object_pose, gripper_config, gripper_pose, quality):
    msg_store = MessageStoreProxy()

    g = Grasp();
    g.object_id = object_id
    g.quality = quality
    g.gripper_config = gripper_config
    g.learned_object_pose = learned_object_pose
    g.gripper_pose = gripper_pose

    try:
        msg_store.insert(g)
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' % e)
        return None


def grasp_import_callback(msg):
    grasps = read_grasps_from_file(msg.grasps_file)
    try:
        for g in grasps:
            pose = PoseStamped()
            pose.header.seq = 1
            pose.header.stamp = rospy.rostime.now()
            pose.header.frame_id = '/world'
            pose.pose.position.x = g[0]
            pose.pose.position.x = g[1]
            pose.pose.position.x = g[2]
            pose.pose.orientation.w = g[3]
            pose.pose.orientation.w = g[4]
            pose.pose.orientation.w = g[5]
            pose.pose.orientation.w = g[6]
            store_grasp(msg.object_id, msg.learned_object_pose, msg.gripper_config, msg.gripper_pose, g[7])    
    except rospy.ROSException, e:
        rospy.logerr('Grasp import failed: %s' % e)
        return GraspImportResponse(False)

    return GraspImportResponse(True)


def read_grasps_from_file(filename):
    grasps = []
    with open(str(filename), 'rb') as f:
        for line in f:
            grasps.append(line)

    return [fromstring(x.replace('\n',''), dtype=float, sep=',') for x in grasps]
