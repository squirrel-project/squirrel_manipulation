import rospy
import actionlib
from squirrel_manipulation_msgs.msg import PtpAction, PtpGoal, JointPtpAction, JointPtpGoal
from squirrel_manipulation_msgs.srv import SoftHandGrasp
from geometry_msgs.msg import Pose, PoseStamped
from pose_estimation_UIBK.srv import *
from pose_estimation_UIBK.msg import *

import tf_lump
import tf

import time, sys

class SoftHand(object):
    def __init__(self):
        'do nothing'

    def run(self):
        try:
            object_id = 'SquirrelBall'
            
            softhand = rospy.ServiceProxy('softhand_grasp', SoftHandGrasp)
            rospy.wait_for_service('softhand_grasp')
            rospy.wait_for_service('pose_recog_UIBK')
            br = tf.TransformBroadcaster()
            transformer = tf.TransformListener()
            
            pose = self.get_object_pose(object_id)
            
            goal_pose = self.stamp_pose(pose, 'kinect_rgb_optical_frame')
            
            br.sendTransform((goal_pose.pose.position.x, goal_pose.pose.position.y, 
              goal_pose.pose.position.z), (goal_pose.pose.orientation.x, 
              goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, 
              goal_pose.pose.orientation.w), 
              rospy.Time.now(), 
              goal_pose.header.frame_id, 
              'kinect_rgb_optical_frame')       
            rospy.sleep(1.0)
            goal_pose.header.stamp = transformer.getLatestCommonTime(goal_pose.header.frame_id, 'origin')            
            goal_pose_origin = transformer.transformPose('origin', goal_pose)
            
          
            goal = PtpGoal()
            goal.pose.position.x = goal_pose_origin.pose.position.x
            goal.pose.position.y = goal_pose_origin.pose.position.y
            goal.pose.position.z = goal_pose_origin.pose.position.z+0.1
            #static orientation
            #goal.pose.orientation.w = -0.00127845082898
            #goal.pose.orientation.x = -0.0458614192903
            #goal.pose.orientation.y = 0.998826861382
            #goal.pose.orientation.z = -0.0154969207942
            goal.pose.orientation.w = 0.73
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.69
            goal.pose.orientation.z = 0.0
                        
            rospy.loginfo("Open hand.")
            softhand(0.0)
           
            rospy.loginfo("Calculating goal.")

            lump_pose = tf_lump.getLumpPose()

            goal_lump = PtpGoal()
            goal_lump.pose.position.x = lump_pose[0] 
            goal_lump.pose.position.y = lump_pose[1]
            goal_lump.pose.position.z = lump_pose[2]*2

            goal_lump.pose.orientation.w = 0.73
            goal_lump.pose.orientation.x = 0.0
            goal_lump.pose.orientation.y = 0.69
            goal_lump.pose.orientation.z = 0.0

            print "UIBK Goal"
            print goal

            print "Lump Tracker Goal"
            print goal_lump

            client = actionlib.SimpleActionClient('cart_ptp', PtpAction)
            client.wait_for_server()

            ch = raw_input('Do you want to continiue (1 for UIBK, 2 for Lump)?')
            if ch  == '1':
              client.send_goal(goal)
            elif ch == '2':
              goal=goal_lump
              client.send_goal(goal)
            else:
              sys.exit("Aborted")
            
            client.wait_for_result()
            result = client.get_result()                
            rospy.loginfo(result.result_status)
            if result.result_status == "execution failed.":
              sys.exit("Execution failed.")

            rospy.loginfo("Grasp.")
            time.sleep(1)
            softhand(0.8)

            rospy.loginfo("Retract.")
            goal.pose.position.z = goal.pose.position.z+0.3
            ch = raw_input('Do you want to continiue (y)?')
            if ch  == 'y':
              client.send_goal(goal)
            else:
              sys.exit("Aborted")

            client.wait_for_result()
            result = client.get_result()                
            rospy.loginfo(result.result_status)
            if result.result_status == "execution failed.":
              softhand(0.0)
              sys.exit("Execution failed.")
        
            rospy.loginfo("Letting loose.")
            softhand(0.0)

            print "Going Home"            
            ch = raw_input('Do you want to continiue (y)?')
            if ch  == 'y':
              client_1 = actionlib.SimpleActionClient('joint_ptp', JointPtpAction)
              client_1.wait_for_server()
              goal_1 = JointPtpGoal()
              goal_1.joints.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
              client_1.send_goal(goal_1)
            
              rospy.loginfo('Good night!')            
            else:
              sys.exit("Aborted")
    
            sys.exit()
        except (rospy.ROSInterruptException, rospy.ServiceException) as e:
            rospy.logerror(e)

            
    def stamp_pose(self, pose, frame_id):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose.position.x = pose.position.x
        pose_stamped.pose.position.y = pose.position.y
        pose_stamped.pose.position.z = pose.position.z
        pose_stamped.pose.orientation.x = pose.orientation.x
        pose_stamped.pose.orientation.y = pose.orientation.y
        pose_stamped.pose.orientation.z = pose.orientation.z
        pose_stamped.pose.orientation.w = pose.orientation.w
        return pose_stamped
        
            
    def get_object_pose(self, obj_id):  
        obj_names = [obj_id]
        
        print('********************')
        print 'Detecting {0}'.format(obj_id)
        #rospy.wait_for_service('pose_recog_UIBK')
        arg_to_obj_detection = object_detect_UIBKRequest()
        
        for obj_name in obj_names:
            obj_to_search = object_pose()
            obj_to_search.id = obj_name
            
            opt_dev_no = 1
            for i in range(opt_dev_no):
                bb_2d = boundingbox_2d()
                bb_2d.boundingbox_2d[0] = 0
                bb_2d.boundingbox_2d[1] = 0
                bb_2d.boundingbox_2d[2] = 639
                bb_2d.boundingbox_2d[3] = 479
                obj_to_search.bb_2dcams.append(bb_2d)#for each cam's bb, in order.
            arg_to_obj_detection.multi_poses.append(obj_to_search)
        try:
            find_objs = rospy.ServiceProxy('pose_recog_UIBK',object_detect_UIBK)
            obj_pose_ret = find_objs(arg_to_obj_detection)
            return obj_pose_ret.detected_poses.multi_poses[0].obj_pose
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e            
            

            

        
