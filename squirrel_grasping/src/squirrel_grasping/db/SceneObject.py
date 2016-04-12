import rospy
from numpy import fromstring
from mongodb_store.message_store import MessageStoreProxy
from squirrel_object_perception_msgs.msg import SceneObject, BCylinder
from squirrel_manipulation_msgs.srv import GraspImportResponse
from geometry_msgs.msg import PoseStamped


def store_grasp(object_id, learned_object_pose, object_cylinder):
    msg_store = MessageStoreProxy()

    g = SceneObject();
    g.header = learned_object_pose.header
    g.bounding_cylinder = object_cylinder
    g.pose = learned_object_pose.pose

    try:
        msg_store.insert_named(object_id,g)
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s' % e)
        return None


if __name__ == '__main__':
    rospy.init_node('scene_object_importer')
    try:
        for g in range(1,2):
            pose = PoseStamped()
            pose.header.seq = 1
            pose.header.stamp = rospy.Time().now()
            pose.header.frame_id = '/map'
            pose.pose.position.x = 0.196
            pose.pose.position.y = -0.343
            pose.pose.position.z = 0.12
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            cylinder = BCylinder()
            cylinder.height = 0.24
            cylinder.diameter = 0.13
            store_grasp( ('o'+str(g)), pose, cylinder)    
    except rospy.ROSException, e:
        rospy.logerr('Grasp import failed: %s' % e)
