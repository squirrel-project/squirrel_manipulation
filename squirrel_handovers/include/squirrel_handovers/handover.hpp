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

#define HANDOVER_NAME "handover"
#define SENSOR_TOPIC "/wrist"
#define FINGERTIP_TOPIC "/fingertips"
#define HAND_SERVICE "/softhand_grasp"

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
    static auto constexpr MIN_VALS_GIVE = 10;	//minimum number of readings per quantity required for the computatio
    static auto constexpr DETECT_THREAS = 5;

    //parameters
    std::string base_frame_;
    std::string wrist_frame_ ;
    std::string robot;
    std::string tuw_robotino;
    std::string uibk_robotino;

    //variables
    int stage;
    bool runHandover_;

    boost::mutex sensor_mutex_;
    std::vector<double> robot_joints_;
    std::vector<double> wrist_sensor_values_;
    std::vector<double> current_forces_ ;
    std::vector<double> current_torques_;
    std::vector<double> fingertip_sensor_values_;

    //sensor
    std::vector<double> force_past;
    std::vector<double> torque_past;


    //subscribers
    ros::Subscriber sub_h, sub_f ;


protected:

    ros::NodeHandle nh, private_nh;

    actionlib::SimpleActionServer<squirrel_manipulation_msgs::HandoverAction> handoverServer;

    squirrel_manipulation_msgs::HandoverFeedback handoverFeedback;

    squirrel_manipulation_msgs::HandoverResult handoverResult;

    void sensorReadCallbackWrist(std_msgs::Float64MultiArray msg);
    void sensorReadCallbackFingers(std_msgs::Float64MultiArray msg);

    std::vector<double> projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose);
    geometry_msgs::Pose tf_stamped2pose(tf::StampedTransform tf_in);
    std::vector<double> projectVectors(double vecX,double vecY,double vecZ,double alpha,double beta,double gamma);

    bool record_magnitude(const std::vector<double>& frc, const std::vector<double>& trq);
    void record_magnitude_give(const std::vector<double>& frc, const std::vector<double>& trq);
    double getMean(const std::vector<double>& starters);
    bool detector();

    void preemptCB();

public:

    HandoverAction(const std::string handoverServerActionName);
    ~HandoverAction();

    void executeHandover(const squirrel_manipulation_msgs::HandoverGoalConstPtr &goal);


};
