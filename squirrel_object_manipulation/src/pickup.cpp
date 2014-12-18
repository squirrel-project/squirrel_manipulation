#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <squirrel_object_manipulation/RobotinoControl.hpp>
#include <squirrel_manipulation_msgs/PickUpAction.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_manipulation/pickup.hpp>

using namespace std;

PickupAction::PickupAction(std::string name) : as(nh, name, boost::bind(&PickupAction::executeCB, this, _1), false), actionName(name) {
    as.start();
}

PickupAction::~PickupAction() {
}

void PickupAction::executeCB(const squirrel_manipulation_msgs::PickUpGoalConstPtr &goal) {

    ros::Rate rate(1);

    // publish info to the console for the user
    ROS_INFO("(pick up action) started pick up of %s for manipulation %s",  goal->object_id.c_str(), goal->next_manipulation.c_str());

    for(int i = 1; i <= 10; ++i) {

        rate.sleep();
        feedback.percent_completed = i * 10;
        as.publishFeedback(feedback);

    }

    result.result_status = "done";
    as.setSucceeded(result);

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "pickup");

  PickupAction pick(ros::this_node::getName());
  ros::spin();

  return 0;

}
