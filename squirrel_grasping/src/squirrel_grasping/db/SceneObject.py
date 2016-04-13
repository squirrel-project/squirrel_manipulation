import rospy
import numpy as np
from std_msgs.msg import Bool
from  numpy import fromstring
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
    rotatory_lock = rospy.Publisher('/twist_mux/locks/pause_rotatory', Bool, queue_size=10)
    rotatory_lock.publish(False)
    try:

        theta = np.linspace(0, 2*np.pi, 6)
        a, b = 0.44 * np.cos(theta), 0.44 * np.sin(theta)
        for g in range(6):

            

            pose = PoseStamped()
            pose.header.seq = 1
            pose.header.stamp = rospy.Time().now()
            pose.header.frame_id = '/odom'
            pose.pose.position.x = a[g]
            pose.pose.position.y = b[g]
            pose.pose.position.z = 0.05 + 2*g
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            cylinder = BCylinder()
            cylinder.height = 0.10 + 4*g
            cylinder.diameter = 0.13
            store_grasp( ('obj'+str(g)), pose, cylinder)

        rotatory_lock.publish(True)
    except rospy.ROSException, e:
        rospy.logerr('Grasp import failed: %s' % e)
        rotatory_lock.publish(True)
