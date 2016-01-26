#include "../include/squirrel_arm_control/SquirrelControlQueue.hpp"

#include <tf/tf.h>
#include <sstream>
#include <ros/ros.h>
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
using namespace arma;
using namespace kukadu;

SquirrelControlQueue::SquirrelControlQueue(double cycleTime, std::string groupName, KUKADU_SHARED_PTR<ros::NodeHandle> node) : ControlQueue(5, cycleTime, KUKADU_SHARED_PTR<Kinematics>(new MoveItKinematics(groupName, "link5"))), baseFrame("/odomp"), targetFrame("/link5"), jointStateTopic("/arm_controller/joint_states") {

    this->node = node;
    this->groupName = groupName;
    currControlType = CONTROLQUEUE_STOP_MODE;

    firstTimeCartFrcReading = true;
    firstTimeJointFrcReading = true;

    group = KUKADU_SHARED_PTR<moveit::planning_interface::MoveGroup>(new moveit::planning_interface::MoveGroup(groupName));
    group->setPlannerId("LBKPIECEkConfigDefault");
    group->setEndEffectorLink("link5");

    jointNames.clear();
    for(int i = 1; i <= 5; ++i) {
        stringstream s;
        s << "joint" << i;
        jointNames.push_back(s.str());
    }

    execution_client = node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");
    planner = KUKADU_SHARED_PTR<trajectory_planner_moveit::TrajectoryPlanner>(new trajectory_planner_moveit::TrajectoryPlanner(*node, *group, jointNames));

    jointPosSub = node->subscribe(jointStateTopic, 10, &SquirrelControlQueue::jointStateCallback, this);

    cartesianStateThread = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&SquirrelControlQueue::retrieveCartJoints, this));

}

SquirrelControlQueue::~SquirrelControlQueue() {
}

void SquirrelControlQueue::setInitValues() {

    ros::Rate sl(5);
    while(currentJointState.n_elem <= 1) {
        ros::spinOnce();
        sl.sleep();
    }

    moveit::core::robotStateToRobotStateMsg(*group->getCurrentState(), jointPlan.trajectory_start);
    jointPlan.group_name = group->getName();
    jointPlan.planning_time = 1.0;
    jt.joint_names = jointNames;

}

void SquirrelControlQueue::jointStateCallback(sensor_msgs::JointState js) {

    jointStateMutex.lock();

        if(js.name.at(0).substr(0, 5).compare(string("joint"))) {
            jointStateMutex.unlock();
            return;
        }

        currentJointStateSjs = js;
        currentJointState = stdToArmadilloVec(js.position);

    jointStateMutex.unlock();

}

void SquirrelControlQueue::submitNextJointMove(arma::vec joints) {

    jointPlan.trajectory_start.joint_state = currentJointStateSjs;

    jt.points.clear();

    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions = vector<double>(armadilloToStdVec(getCurrentJoints().joints));
    jtp.time_from_start = dur.data.fromSec(0);
    jt.points.push_back(jtp);

    jtp.positions = vector<double>(armadilloToStdVec(joints));
    jtp.time_from_start = dur.data.fromSec(getTimeStep());

    jt.points.push_back(jtp);

    jointPlan.trajectory.joint_trajectory = jt;
    planner->executePlan(jointPlan, execution_client);

}

void SquirrelControlQueue::submitNextCartMove(geometry_msgs::Pose pose) {

    ROS_WARN("(SquirrelControlQueue) cartesian command mode is not supported yet");

}

void SquirrelControlQueue::setCurrentControlTypeInternal(int controlType) {

    currControlType = controlType;

}

void SquirrelControlQueue::jointPtpInternal(arma::vec joints) {

    moveit_msgs::MotionPlanResponse jointPlan;
    vector<double> jointsVector = armadilloToStdVec(joints);
    planner->plan(jointsVector, jointPlan);
    planner->executePlan(jointPlan, execution_client);

}

void SquirrelControlQueue::cartPtpInternal(geometry_msgs::Pose pose) {

    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.pose = pose;

    moveit_msgs::MotionPlanResponse plan;
    if(planner->plan(ps, plan)) {

        moveit_msgs::RobotState state;

        state.joint_state.name = plan.trajectory.joint_trajectory.joint_names;
        state.joint_state.position = plan.trajectory.joint_trajectory.points.back().positions;

        ros::ServiceClient execution_client = node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");

        planner->executePlan(plan, execution_client);

    } else {
        ROS_ERROR("(SquirrelControlQueue) planning for goal failed!");
    }


}

void SquirrelControlQueue::setJntPtpThresh(double thresh) {

    ROS_WARN("(SquirrelControlQueue) joint ptp threshold setting is not supported yet");

}

void SquirrelControlQueue::setAdditionalLoad(float loadMass, float loadPos) {

    ROS_WARN("(SquirrelControlQueue) additional load setting is not supported yet");

}

void SquirrelControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {

    ROS_WARN("(SquirrelControlQueue) stiffness settings are not supported yet");

}

int SquirrelControlQueue::getCurrentControlType() {

    return currControlType;

}

geometry_msgs::PoseStamped SquirrelControlQueue::tf_stamped2pose_stamped(tf::StampedTransform tf_in) {

    geometry_msgs::PoseStamped emap;

    emap.pose.position.x = tf_in.getOrigin().x();
    emap.pose.position.y = tf_in.getOrigin().y();
    emap.pose.position.z = tf_in.getOrigin().z();;
    emap.pose.orientation.x = tf_in.getRotation().x();
    emap.pose.orientation.y = tf_in.getRotation().y();
    emap.pose.orientation.z = tf_in.getRotation().z();;
    emap.pose.orientation.w = tf_in.getRotation().w();;
    emap.header.frame_id = tf_in.frame_id_;

    return emap;

}

void SquirrelControlQueue::retrieveCartJoints() {

    try {
        tf::StampedTransform trans;
        tfList.waitForTransform(baseFrame, targetFrame, ros::Time::now(), ros::Duration(0.02));
        tfList.lookupTransform(baseFrame, targetFrame, ros::Time(0), trans);
        currentPose = tf_stamped2pose_stamped(trans).pose;
    } catch (tf::TransformException& ex) {

    }

}

geometry_msgs::Pose SquirrelControlQueue::getCurrentCartesianPose() {

    return currentPose;

}

mes_result SquirrelControlQueue::getCurrentJoints() {

    mes_result ret;
    ret.time = getCurrentTime();
    jointStateMutex.lock();
        ret.joints = currentJointState;
    jointStateMutex.unlock();
    return ret;

}

mes_result SquirrelControlQueue::getCurrentJntFrcTrq() {

    if(firstTimeJointFrcReading) {
        firstTimeJointFrcReading = false;
        ROS_WARN("(SquirrelControlQueue) reading joint force values not supported yet");
    }

    mes_result ret;
    ret.time = getCurrentTime();
    ret.joints = vec(getMovementDegreesOfFreedom());
    // forces currently not supported (this line has to be replaced)
    ret.joints.fill(0.0);
    return ret;

}

mes_result SquirrelControlQueue::getCurrentCartesianFrcTrq() {

    if(firstTimeCartFrcReading) {
        firstTimeCartFrcReading = false;
        ROS_WARN("(SquirrelControlQueue) reading cartesian force values not supported yet");
    }

    mes_result ret;
    ret.time = getCurrentTime();
    ret.joints = vec(6);
    // forces currently not supported (this line has to be replaced)
    ret.joints.fill(0.0);
    return ret;

}

std::string SquirrelControlQueue::getRobotName() {
    return groupName;
}

std::string SquirrelControlQueue::getRobotFileName() {
    return groupName;
}

std::vector<std::string> SquirrelControlQueue::getJointNames() {
    return jointNames;
}

void SquirrelControlQueue::safelyDestroy() {

}

bool SquirrelControlQueue::stopQueueWhilePtp() {

}
