#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
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



class HandoverAction {
private:
    static auto constexpr  SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR = -2.27;

    //parameters
    std::string base_frame_;
    std::string wrist_frame_ ;
    std::string robot;
    std::string tuw_robotino;
    std::string uibk_robotino;

    //variables
    int stage;

    boost::mutex sensor_mutex_;
    std::vector<double> robot_joints_;
    std::vector<double> wrist_sensor_values_;
    std::vector<double> fingertip_sensor_values_;


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



public:

    HandoverAction(const std::string handoverServerActionName);
    ~HandoverAction();

    void executeHandover(const squirrel_manipulation_msgs::HandoverGoalConstPtr &goal);


};
