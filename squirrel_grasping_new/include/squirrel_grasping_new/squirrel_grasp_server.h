#ifndef SQUIRREL_GRASP_SERVER_H_
#define SQUIRREL_GRASP_SERVER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_manipulation_msgs/BlindGraspAction.h>
#include <squirrel_motion_planner_msgs/PlanEndEffector.h>
#include <squirrel_motion_planner_msgs/PlanPose.h>
#include <squirrel_motion_planner_msgs/SendControlCommand.h>
#include <squirrel_motion_planner_msgs/UnfoldArm.h>

#define METAHAND_STRING_ "metahand"
#define SOFTHAND_STRING_ "softhand"
#define PLANNING_FRAME_ "map"

/**
 * \brief Grasp server
 * \author Tim Patten (patten@acin.tuwien.ac.at)
 */
class SquirrelGraspServer
{
public:

  enum HandType
  {
    UNKNOWN = 0,
    METAHAND = 1,
    SOFTHAND = 2
  };

  /**
   * \brief Constructor with ros node handle
   */
  SquirrelGraspServer ( ros::NodeHandle &n, const std::string &action_name );

  /**
   * \brief Destructor
   */
  virtual
  ~SquirrelGraspServer ();

  /**
   * \brief Initialize the server with hand type and find servers and topics
   */
  bool initialize ( const std::string &action_name );

private:

  // Node handle
  ros::NodeHandle *n_;
  // Action server
  actionlib::SimpleActionServer<squirrel_manipulation_msgs::BlindGraspAction> as_;
  // Action name
  std::string action_name_;
  // Hand type
  HandType hand_type_;
  // Messages to publish feedback and result
  squirrel_manipulation_msgs::BlindGraspFeedback feedback_;
  squirrel_manipulation_msgs::BlindGraspResult result_;

  // Services
  ros::ServiceClient *arm_unfold_client_;
  ros::ServiceClient *arm_end_eff_planner_client_;
  ros::ServiceClient *arm_pose_planner_client_;
  ros::ServiceClient *arm_send_trajectory_client_;
  ros::ServiceClient *hand_client_;

  squirrel_motion_planner_msgs::UnfoldArm unfold_goal_;
  squirrel_motion_planner_msgs::PlanEndEffector end_eff_goal_;
  squirrel_motion_planner_msgs::PlanPose pose_goal_;
  squirrel_motion_planner_msgs::SendControlCommand cmd_goal_;

  // Transform
  tf::TransformListener tf_listener_;

  std::vector<double> folded_pose;
  std::vector<double> unfolded_pose;

  void graspCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

  bool metahandCallBack ( const geometry_msgs::PoseStamped &goal );

  bool softhandCallBack ( const geometry_msgs::PoseStamped &goal );

  bool transformPose ( const std::string &origin_frame, const std::string &target_frame,
                      geometry_msgs::PoseStamped &in, geometry_msgs::PoseStamped &out );

};

#endif

