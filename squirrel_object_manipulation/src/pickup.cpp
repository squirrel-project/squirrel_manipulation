#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <fstream>

#include <squirrel_object_manipulation/pickup.hpp>

using namespace std;

ManipulationAction::ManipulationAction(const std::string pickupServerActionName, const std::string blindServerActionName, const std::string dropServerActionName, const std::string putServerActionName, const std::string inspectServerActionName, const std::string leanServerActionName) :
                            pickServer(nh, pickupServerActionName, boost::bind(&ManipulationAction::executePickUp, this, _1), false),
                            blindServer(nh, blindServerActionName, boost::bind(&ManipulationAction::executeBlindGrasp, this, _1)),
                            dropServer(nh, dropServerActionName, boost::bind(&ManipulationAction::executeDrop, this, _1)),
                            putServer(nh, putServerActionName, boost::bind(&ManipulationAction::executePut, this, _1)),
                            inspectServer(nh, inspectServerActionName, boost::bind(&ManipulationAction::executeInspect, this, _1)),
                            leanServer(nh, leanServerActionName, boost::bind(&ManipulationAction::executeLean, this, _1))
{

    pickServer.start();
    blindServer.start();
    dropServer.start();
    putServer.start();

}

ManipulationAction::~ManipulationAction() {
}

void ManipulationAction::executeInspect(const squirrel_manipulation_msgs::InspectGoalConstPtr &goal) {

    ros::Rate rate(1);

    // publish info to the console for the user
    ROS_INFO("(inspect action) started inspection");

    for(int i = 1; i <= 10; ++i) {

        rate.sleep();
        inspectFeedback.percent_completed = i * 10;
        inspectServer.publishFeedback(inspectFeedback);

    }

    inspectResult.result_status = "done";
    inspectServer.setSucceeded(inspectResult);

}

void ManipulationAction::executeLean(const squirrel_manipulation_msgs::LeanGoalConstPtr &goal) {

    ros::Rate rate(1);

    // publish info to the console for the user
    ROS_INFO("(drop action) started leaning");

    for(int i = 1; i <= 10; ++i) {

        rate.sleep();
        leanFeedback.percent_completed = i * 10;
        leanServer.publishFeedback(leanFeedback);

    }

    leanResult.result_status = "done";
    leanServer.setSucceeded(leanResult);

}

void ManipulationAction::executeDrop(const squirrel_manipulation_msgs::DropGoalConstPtr &goal) {

    ros::Rate rate(1);

    // publish info to the console for the user
    ROS_INFO("(drop action) dropping object at destination %s",  goal->destination_id.c_str());

    for(int i = 1; i <= 10; ++i) {

        rate.sleep();
        dropFeedback.percent_completed = i * 10;
        dropServer.publishFeedback(dropFeedback);

    }

    dropResult.result_status = "done";
    dropServer.setSucceeded(dropResult);


}

void ManipulationAction::executePut(const squirrel_manipulation_msgs::PutDownGoalConstPtr &goal) {

    ros::Rate rate(1);

    // publish info to the console for the user
    ROS_INFO("(put down action) putting down object at destination %s",  goal->destination_id.c_str());

    for(int i = 1; i <= 10; ++i) {

        rate.sleep();
        putFeedback.percent_completed = i * 10;
        putServer.publishFeedback(putFeedback);

    }

    putResult.result_status = "done";
    putServer.setSucceeded(putResult);


}

void ManipulationAction::executePickUp(const squirrel_manipulation_msgs::PickUpGoalConstPtr &goal) {

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

void ManipulationAction::executeBlindGrasp(const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal) {

    ros::Rate rate(1);

    // publish info to the console for the user
    ROS_INFO("(pick up action) performing blind grasp");

    for(int i = 1; i <= 10; ++i) {

        rate.sleep();
        blindFeedback.percent_completed = i * 10;
        blindServer.publishFeedback(blindFeedback);

    }

    blindResult.result_status = "done";
    blindServer.setSucceeded(blindResult);

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "manipulation");

    ManipulationAction pick(SQRL_PICKUP_NAME, SQRL_BLIND_GRASP_NAME, SQRL_DROP_NAME, SQRL_PUT_DOWN_NAME, SQRL_INSPECT_NAME, SQRL_LEAN_NAME);
    ros::spin();

    return 0;

}
