#include <ros/ros.h>
#include <iostream>

#include <actionlib/server/simple_action_server.h>

#include <squirrel_manipulation_msgs/HandoverAction.h>
#include <squirrel_manipulation_msgs/HandoverActionFeedback.h>
#include <squirrel_manipulation_msgs/HandoverActionGoal.h>
#include <squirrel_manipulation_msgs/HandoverActionResult.h>

#define HANDOVER_NAME "handover"


class HandoverAction {
 private:
  
 protected:

  ros::NodeHandle nh, private_nh;

  actionlib::SimpleActionServer<squirrel_manipulation_msgs::HandoverAction> handoverServer;

  squirrel_manipulation_msgs::HandoverFeedback handoverFeedback;

  squirrel_manipulation_msgs::HandoverResult handoverResult;

public:

    HandoverAction(const std::string handoverServerActionName);

    void executeHandover(const squirrel_manipulation_msgs::HandoverGoalConstPtr &goal);


};
