import rospy
import actionlib
from squirrel_manipulation_msgs.msg import PtpAction
from squirrel_manipulation_msgs.srv import SoftHandGrasp

class Test(object):
    def __init__(self):
        'do nothing'

    def run(self):
        try:
            client = actionlib.SimpleActionClient('cart_ptp', PtpAction)
            client.wait_for_server()
            goal = PtpAction()
            goal.pose.point.x = 0
            goal.pose.point.y = 0
            goal.pose.point.z = 0
            goal.pose.orientation.w = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            
            client.send_goal(goal)
            
            client.wait_for_result()
            
            result = client.get_result()
            
            rospy.loginfo(result.result_status)
            
            rospy.wait_for_service('softhand_grasp')
            
            softhand = rospy.ServiceProxy('softhand_grasp', SoftHandGrasp)
            softhand(0.8)
                        
            # is it centimeters?
            goal.pose.point.z = goal.pose.point.z-0.15

            client.send_goal(goal)
            
            client.wait_for_result()
            
            result = client.get_result()
            
            rospy.loginfo(result.result_status)

            softhand(0.0)

            rospy.loginfo('quitting...')
            
            
        except (rospy.ROSInterruptedException, rospy.ServiceException) as e:
            rospy.logerror(e)

            

        
