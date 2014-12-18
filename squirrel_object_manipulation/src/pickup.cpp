#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <fstream>

#include <squirrel_object_manipulation/pickup.hpp>

using namespace std;

PickupAction::PickupAction(const std::string std_pickupServerActionName, const std::string std_blindServerActionName) :
                            pickServer(nh, std_pickupServerActionName, boost::bind(&PickupAction::executePickUp, this, _1), false),
                            blindServer(nh, std_blindServerActionName, boost::bind(&PickupAction::executeBlindGrasp, this, _1))
{
    pickServer.start();
    blindServer.start();
}

PickupAction::~PickupAction() {
}

void PickupAction::executePickUp(const squirrel_manipulation_msgs::PickUpGoalConstPtr &goal) {

    ros::Rate rate(1);

    // publish info to the console for the user
    ROS_INFO("(pick up action) started pick up of %s for manipulation %s",  goal->object_id.c_str(), goal->next_manipulation.c_str());

    for(int i = 1; i <= 10; ++i) {

        rate.sleep();
        pickupFeedback.percent_completed = i * 10;
        pickServer.publishFeedback(pickupFeedback);

    }

    pickupResult.result_status = "done";
    pickServer.setSucceeded(pickupResult);

}

void PickupAction::executeBlindGrasp(const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal) {

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "manipulation");

    PickupAction pick(PICKUP_NAME, BLIND_GRASP_NAME);
    ros::spin();

    return 0;

}
