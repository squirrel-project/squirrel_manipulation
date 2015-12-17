#include <vector>
#include <iostream>
#include <std_msgs/Duration.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/transforms/transforms.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
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
    group.setPlannerId("LBKPIECEkConfigDefault");
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

    ros::ServiceClient execution_client = node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");
    trajectory_planner_moveit::TrajectoryPlanner planner(*node, group, jointNames);

    geometry_msgs::PoseStamped newPose = group.getCurrentPose();
    cout << "current pose: " << newPose << endl;

    // this works fine (joint planning and movement)
    // waving behaviour
    moveit_msgs::MotionPlanResponse jointPlan;
    moveit::core::robotStateToRobotStateMsg(*group.getCurrentState(), jointPlan.trajectory_start);
    jointPlan.group_name = group.getName();
    jointPlan.planning_time = 1.0;
    trajectory_msgs::JointTrajectory jt;
    std_msgs::Duration dur;

    jt.joint_names = jointNames;

    for(int j = 0; j < 200; ++j) {

        jt.points.clear();

        trajectory_msgs::JointTrajectoryPoint jtp;
        jtp.positions = vector<double>(jointVals);
        jtp.time_from_start = dur.data.fromSec(0);
        jt.points.push_back(jtp);
        for(int i = 1; i < 2; ++i) {

            jtp.positions = vector<double>(jointVals);
            jtp.time_from_start = dur.data.fromSec(i * 0.04);

            jt.points.push_back(jtp);

            int sign = 1;
            jointVals.at(0) += sign * 0.005;
            //sign *= -1;

        }

        if(j % 20 == 0)  {
            Eigen::Vector3d refPoint;
            refPoint << 0.0, 0.0, 0.0;
            Eigen::MatrixXd jacob;
            moveit::core::RobotStatePtr currStat = group.getCurrentState();
            const moveit::core::JointModelGroup* jmg = currStat->getJointModelGroup(string("Arm"));
            // last boolean determines wheter the jacobian should be represented in quaternion representation
            currStat->getJacobian(jmg, jmg->getLinkModel(string("link5")), refPoint, jacob, false);
            cout << "jacobian: " << endl << jacob << endl << endl << endl;
        }

        jointPlan.trajectory.joint_trajectory = jt;
        planner.executePlan(jointPlan, execution_client);

    }


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
