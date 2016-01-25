#include <ros/ros.h>
#include <iostream>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <actionlib/server/simple_action_server.h>


#include <squirrel_manipulation_msgs/SmashAction.h>
#include <squirrel_manipulation_msgs/SmashActionFeedback.h>
#include <squirrel_manipulation_msgs/SmashActionGoal.h>
#include <squirrel_manipulation_msgs/SmashActionResult.h>

#include <squirrel_object_manipulation/RobotinoControl.hpp>
#include <squirrel_object_manipulation/conversion_utils.hpp>



#define SMASH_NAME "smash"


class SmashAction {
 private:
  geometry_msgs::Pose2D pose_m_;

  ros::Subscriber pose_sub_;

  RobotinoControl *robotino; 

  void updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& );

  std::string pose_topic_;
  

 protected:

  ros::NodeHandle nh, private_nh;

  actionlib::SimpleActionServer<squirrel_manipulation_msgs::SmashAction> smashServer;

  squirrel_manipulation_msgs::SmashFeedback smashFeedback;

  squirrel_manipulation_msgs::SmashResult smashResult;

public:

    SmashAction(const std::string smashServerActionName);
    ~SmashAction();

    void executeSmash(const squirrel_manipulation_msgs::SmashGoalConstPtr &goal);



};
