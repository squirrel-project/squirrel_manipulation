#include <vector>
#include <iostream>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <moveit/move_group_interface/move_group.h>
#include <uibk_planning_node/PathTrajectoryPlanner.h>
#include <moveit_msgs/ExecuteKnownTrajectoryRequest.h>

using namespace std;

int main(int argc, char** args) {

    ros::init(argc, args, "robotino_demo"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);

    string groupName = "Arm";

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Retrieving joint state...");
    moveit::planning_interface::MoveGroup group(groupName);
    group.setEndEffectorLink("link5");

    vector<double> jointVals = group.getCurrentJointValues();
    for(int i = 0; i < jointVals.size(); ++i) {
        cout << jointVals.at(i) << " ";
    }

    ROS_INFO("Connecting to planning service...");

    vector<string> jointNames;
    for(int i = 0; i < 5; ++i) {
        stringstream s;
        s << "joint" << (i + 1);
        jointNames.push_back(s.str());
    }

    trajectory_planner_moveit::TrajectoryPlanner planner(*node, "Arm", jointNames);

    geometry_msgs::PoseStamped newPose = group.getCurrentPose();
    cout << "current pose: " << newPose << endl;

    /*
    // this works fine (joint planning and movement)
    moveit_msgs::MotionPlanResponse jointPlan;
    if(planner.plan(jointVals, jointPlan)) {

        ROS_INFO("Plan to goal1 found");
        moveit_msgs::RobotState state;

        state.joint_state.name = jointPlan.trajectory.joint_trajectory.joint_names;
        state.joint_state.position = jointPlan.trajectory.joint_trajectory.points.back().positions;

        ros::ServiceClient execution_client = node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");

        planner.executePlan(jointPlan, execution_client);

    } else {
        ROS_ERROR("Planning for goal1 failed!");
    }
    */

    ROS_INFO("joint execution done...press enter to continue...");
    getchar();

    /*
    // i tried this with, as well as different small values and the original position
    newPose.pose.position.x += 0.1;
    newPose.pose.position.z -= 0.5;
    */

	// this is another position that is valid
    newPose.pose.position.x = -0.109621;
    newPose.pose.position.y = 0.19786;
    newPose.pose.position.z = 0.699962;
    newPose.pose.orientation.x = -0.642689;
    newPose.pose.orientation.y = 0.0108443;
    newPose.pose.orientation.z = 0.326966;
    newPose.pose.orientation.w = 0.692768;

    moveit_msgs::MotionPlanResponse plan;

/*
    // approach 2 --> uses the cartesian path service directly
    trajectory_planner_moveit::PathTrajectoryPlanner p2(*node, "Arm", jointNames, "link5");
    p2.plan(newPose, plan);
*/

	// approach 1 --> computes inverse kinematics explicitely and tries to execute it
    if(planner.plan(newPose, plan)) {

        ROS_INFO("Plan to goal1 found");
        moveit_msgs::RobotState state;

        state.joint_state.name = plan.trajectory.joint_trajectory.joint_names;
        state.joint_state.position = plan.trajectory.joint_trajectory.points.back().positions;

        ros::ServiceClient execution_client = node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");

        planner.executePlan(plan, execution_client);

    } else {
        ROS_ERROR("Planning for goal1 failed!");
    }

    getchar();

    return 0;

}
