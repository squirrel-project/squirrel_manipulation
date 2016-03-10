#ifndef SQUIRREL_BASE_CONTROL_QUEUE
#define SQUIRREL_BASE_CONTROL_QUEUE

#include <string>
#include <vector>
#include <armadillo>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <kukadu/kukadu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class SquirrelBaseControlQueue : public kukadu::ControlQueue {

private:

    bool currentBumper;

    geometry_msgs::Pose currentPose;

    std::vector<std::string> jointNames;

    const std::string moveTopic;
    const std::string poseTopic;
    const std::string bumperTopic;

    ros::Subscriber subBumper;
    ros::Subscriber subPose;

    ros::Publisher pubMove;

    kukadu_mutex poseMutex;

    KUKADU_SHARED_PTR<ros::NodeHandle> node;

    void callbackBumper(std_msgs::Bool msg);
    void updatePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);

protected:

    virtual void setInitValues();
    virtual void jointPtpInternal(arma::vec joints);
    virtual void submitNextJointMove(arma::vec joints);
    virtual void cartPtpInternal(geometry_msgs::Pose pose);
    virtual void submitNextCartMove(geometry_msgs::Pose pose);
    virtual void setCurrentControlTypeInternal(int controlType);

public:

    SquirrelBaseControlQueue(double cycleTime, KUKADU_SHARED_PTR<ros::NodeHandle> node);
    ~SquirrelBaseControlQueue();

    virtual void setJntPtpThresh(double thresh);
    virtual void setAdditionalLoad(float loadMass, float loadPos);
    virtual void setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq);

    virtual bool stopQueueWhilePtp();

    virtual int getCurrentControlType();

    virtual geometry_msgs::Pose getCurrentCartesianPose();

    virtual mes_result getCurrentJoints();
    virtual mes_result getCurrentJntFrcTrq();
    virtual mes_result getCurrentCartesianFrcTrq();

    virtual std::string getRobotName();
    virtual std::string getRobotFileName();
    virtual std::vector<std::string> getJointNames();

    virtual std::vector<arma::vec> computeIk(geometry_msgs::Pose targetPose);
    virtual geometry_msgs::Pose computeFk(std::vector<double> joints);

    virtual void safelyDestroy();

};

#endif
