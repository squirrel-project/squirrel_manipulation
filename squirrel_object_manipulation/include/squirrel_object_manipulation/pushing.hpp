#include <squirrel_object_manipulation/RobotinoControl.hpp>
#include <squirrel_rgbd_mapping_msgs/GetPushingPlan.h>
#include <squirrel_manipulation_msgs/PushAction.h>
#include <squirrel_manipulation_msgs/PushActionFeedback.h>
#include <squirrel_manipulation_msgs/PushActionGoal.h>
#include <squirrel_manipulation_msgs/PushActionResult.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "squirrel_object_manipulation/RobotinoControl.hpp"

#define PUSH_NAME "push"


class PushAction {
 private:
  geometry_msgs::Pose2D pose_m_;

  ros::Subscriber pose_sub_;

  RobotinoControl *robotino; 

  void updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& );

  std::string pose_topic_;

 protected:

  ros::NodeHandle nh, private_nh;

  actionlib::SimpleActionServer<squirrel_manipulation_msgs::PushAction> pushServer;

  squirrel_manipulation_msgs::PushFeedback pushFeedback;

  squirrel_manipulation_msgs::PushResult pushResult;

public:

    PushAction(const std::string pushServerActionName);
    ~PushAction();

    void executePush(const squirrel_manipulation_msgs::PushGoalConstPtr &goal);



};
