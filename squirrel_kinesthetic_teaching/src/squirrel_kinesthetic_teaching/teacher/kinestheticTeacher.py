import rospy
import sys
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, EmptyResponse
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, roscpp_initialize
from geometry_msgs.msg import Pose, PoseStamped


class KinestheticTeacher:
    def __init__(self):
        while rospy.get_time() == 0.0: pass
        rospy.loginfo(rospy.get_caller_id() + ': starting KinestheticTeacher')
        self.teaching = False
        self.delta = 0.01
        self.rate = rospy.Rate(10)
        self.start = rospy.Service('/squirrel_manipulation/start_teaching', Empty, self.start)
        self.start = rospy.Service('/squirrel_manipulation/stop_teaching', Empty, self.stop)
        self.wrist_sub = rospy.Subscriber('/wrist', Float64MultiArray, self.follow, queue_size=1);
        roscpp_initialize(sys.argv)

        self.group = None


    def start(self, msg):
        if not self.teaching:
            rospy.loginfo(rospy.get_caller_id() + ': entering teaching mode')
            self.teaching = True
            self.group = MoveGroupCommander('Arm')            
            return EmptyResponse()
        else:
            rospy.logwarn(rospy.get_caller_id() + ': already in teaching mode')
            return EmptyResponse()
                
        
    def stop(self, msg):
        if self.teaching:
            rospy.loginfo(rospy.get_caller_id() + ': leaving teaching mode')
            self.teaching = False
            self.group = None            
            return EmptyResponse()
        else:
            rospy.logwarn(rospy.get_caller_id() + ': not in teaching mode')
            return EmptyResponse()


    def follow(self, msg):        
        while not rospy.is_shutdown() and self.teaching:
            self.group.clear_pose_targets()
            rospy.loginfo("while...")
            current_pose = self.group.get_current_pose()
            target_pose = Pose()
                        
            target_pose.position.x = current_pose.pose.position.x + self.delta * msg.data[0]
            target_pose.position.y = current_pose.pose.position.y + self.delta * msg.data[1]
            target_pose.position.z = current_pose.pose.position.z + self.delta * msg.data[2]
            #target_pose.orientation.w = current_pose.pose.orientation.w + delta * msg.data[3];
            #target_pose.orientation.x = current_pose.pose.orientation.x + delta * msg.data[4];
            #target_pose.orientation.y = current_pose.pose.orientation.y + delta * msg.data[5];
            #target_pose.orientation.z = current_pose.pose.orientation.z + delta * msg.data[6];
            target_pose.orientation.w = 1.0
                    
            rospy.loginfo(rospy.get_caller_id() + ': target_pose: \n %s', target_pose)
            
            self.group.set_pose_target(target_pose)
            self.group.plan()
            self.group.go(wait=True)
        




