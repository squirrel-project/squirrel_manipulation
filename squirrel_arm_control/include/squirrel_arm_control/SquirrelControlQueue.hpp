#include <string>
#include <vector>
#include <armadillo>
#include <ros/ros.h>
#include <kukadu/kukadu.h>
#include <std_msgs/Duration.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread/mutex.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>

class SquirrelControlQueue : public ControlQueue {

private:

    const std::string baseFrame;
    const std::string targetFrame;
    const std::string jointStateTopic;

    bool firstTimeCartFrcReading;
    bool firstTimeJointFrcReading;

    int currControlType;

    std::string groupName;

    std::vector<std::string> jointNames;
    \
    arma::vec currentJointState;
    sensor_msgs::JointState currentJointStateSjs;

    geometry_msgs::Pose currentPose;

    tf::TransformListener tfList;

    KUKADU_SHARED_PTR<ros::NodeHandle> node;
    KUKADU_SHARED_PTR<moveit::planning_interface::MoveGroup> group;
    KUKADU_SHARED_PTR<trajectory_planner_moveit::TrajectoryPlanner> planner;

    boost::mutex jointStateMutex;

    ros::Subscriber jointPosSub;

    ros::ServiceClient execution_client;

    moveit_msgs::MotionPlanResponse jointPlan;

    trajectory_msgs::JointTrajectory jt;

    std_msgs::Duration dur;

    void jointStateCallback(sensor_msgs::JointState js);
    geometry_msgs::PoseStamped tf_stamped2pose_stamped(tf::StampedTransform tf_in);

protected:

    virtual void setInitValues();
    virtual void jointPtpInternal(arma::vec joints);
    virtual void submitNextJointMove(arma::vec joints);
    virtual void cartPtpInternal(geometry_msgs::Pose pose);
    virtual void submitNextCartMove(geometry_msgs::Pose pose);
    virtual void setCurrentControlTypeInternal(int controlType);

public:

    SquirrelControlQueue(double cycleTime, std::string groupName, KUKADU_SHARED_PTR<ros::NodeHandle> node);
    ~SquirrelControlQueue();

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

    virtual void safelyDestroy();

};
