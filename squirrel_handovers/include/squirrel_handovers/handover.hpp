#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <cmath>
#include <math.h>
#include <algorithm>
#include <cassert>

#include <boost/thread.hpp>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>

#include <kukadu/kukadu.hpp>
#include <squirrel_manipulation_msgs/SoftHandGrasp.h>
#include <kclhand_control/ActuateHandAction.h>
#include <kclhand_control/ActuateHandActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <squirrel_manipulation_msgs/HandoverAction.h>
#include <squirrel_manipulation_msgs/HandoverActionFeedback.h>
#include <squirrel_manipulation_msgs/HandoverActionGoal.h>
#include <squirrel_manipulation_msgs/HandoverActionResult.h>

#include <squirrel_manipulation_msgs/JointPtpAction.h>
#include <squirrel_manipulation_msgs/JointPtpActionFeedback.h>
#include <squirrel_manipulation_msgs/JointPtpActionGoal.h>
#include <squirrel_manipulation_msgs/JointPtpActionResult.h>

#include <sensor_msgs/JointState.h>





#define HANDOVER_NAME "handover"
#define SENSOR_TOPIC "/wrist"
#define FINGERTIP_TOPIC "/fingertips"
#define HAND_SERVICE "/softhand_grasp"
#define SAFETY_TOPIC "/reset_safety"
#define JOINTS_TOPIC "/real/robotino/joint_control/get_state"
#define TILT_TOPIC "/tilt_controller/command"
#define PAN_TOPIC "/pan_controller/command"


enum Axes
{
    X,
    Y,
    Z,
    AxesNum
};

class HandoverAction {
private:
    static auto constexpr  SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR = -2.27; //displacement  of forces and torques
    static auto constexpr MIN_VALS = 3;	//minimum number of readings per quantity required for the computatio
    static auto constexpr MIN_VALS_GIVE = 6;	//minimum number of readings per quantity required for the computation

    //parameters
    std::string base_frame_;
    std::string wrist_frame_ ;
    std::string robot;
    std::string tuw_robotino;
    std::string uibk_robotino;
    double take_threshold_;



    //variables
    int stage;
    bool runHandover_;
    double handover_frequency_;

    boost::mutex sensor_mutex_;
		
    std::vector<double> robot_joints_;
    std::vector<double> wrist_sensor_values_;
    std::vector<double> current_forces_ ;
    std::vector<double> current_torques_;
    std::vector<double> fingertip_sensor_values_;
    std::vector<double> joint_values_;

    //sensor
    std::vector<double> force_past;
    std::vector<double> torque_past;

    //subscribers
    ros::Subscriber sub_h, sub_f ;
    ros::Subscriber sub_joints_;
    //publishers
    ros::Publisher safety_pub_;
    ros::Publisher tiltPub;
    ros::Publisher panPub;


protected:

    ros::NodeHandle nh, private_nh;

    actionlib::SimpleActionServer<squirrel_manipulation_msgs::HandoverAction> handoverServer;

    squirrel_manipulation_msgs::HandoverFeedback handoverFeedback;

    squirrel_manipulation_msgs::HandoverResult handoverResult;

    void sensorReadCallbackWrist(std_msgs::Float64MultiArray msg);
    void sensorReadCallbackFingers(std_msgs::Float64MultiArray msg);
    void callbackJoints(sensor_msgs::JointState msg);

    bool record_magnitude(const std::vector<double>& frc, const std::vector<double>& trq);
    void record_magnitude_simple(const std::vector<double>& frc, const std::vector<double>& trq);
    double getMean(const std::vector<double>& starters);
    bool detector_take();
    bool detector_give(); 

    void moveTilt(double val);
    void movePan(double val);
    void resetSafety();


    void preemptCB();

public:

    HandoverAction(const std::string handoverServerActionName);
    ~HandoverAction();

    void executeHandover(const squirrel_manipulation_msgs::HandoverGoalConstPtr &goal);


};
