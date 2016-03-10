#include <ros/ros.h>
#include <iostream>
#include <armadillo>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>

#include <squirrel_manipulation_msgs/SmashAction.h>
#include <squirrel_manipulation_msgs/SmashActionFeedback.h>
#include <squirrel_manipulation_msgs/SmashActionGoal.h>
#include <squirrel_manipulation_msgs/SmashActionResult.h>

#include <squirrel_object_manipulation/RobotinoControl.hpp>
#include <squirrel_object_manipulation/conversion_utils.hpp>
#include <squirrel_object_manipulation/math_utils.hpp>


#define SMASH_NAME "smash"


class SmashAction {
 private:
  geometry_msgs::Pose2D pose_robot_;

  ros::Subscriber pose_sub_;
  ros::Publisher marker_target_c_;

  RobotinoControl *robotino; 

  void updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& );

  std::string pose_topic_;
  double robot_diameter_, max_distance_, end_toll_, angle_toll_, attraction_coefficient_,rotation_coefficient_, vel_ang_max_, vel_lin_max_, vel_lin_back_max_;
  bool shake_;
  //~SmashAction();

 protected:

  ros::NodeHandle nh, private_nh;

  actionlib::SimpleActionServer<squirrel_manipulation_msgs::SmashAction> smashServer;

  squirrel_manipulation_msgs::SmashFeedback smashFeedback;

  squirrel_manipulation_msgs::SmashResult smashResult;

public:

    SmashAction(const std::string smashServerActionName);


    void executeSmash(const squirrel_manipulation_msgs::SmashGoalConstPtr &goal);
    void publishMarker(double goalx, double goaly);



};
