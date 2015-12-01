#ifndef SQUIRREL_OBJECT_MANIPULATION_PUSH_ACTION_H_
#define SQUIRREL_OBJECT_MANIPULATION_PUSH_ACTION_H_

#include <ros/ros.h>
#include <iostream>

#include <std_msgs/Float64.h>
#include <boost/assert.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/range/numeric.hpp>

#include <squirrel_object_manipulation/RobotinoControl.hpp>
#include <squirrel_rgbd_mapping_msgs/GetPushingPlan.h>
#include <squirrel_manipulation_msgs/PushAction.h>
#include <squirrel_manipulation_msgs/PushActionFeedback.h>
#include <squirrel_manipulation_msgs/PushActionGoal.h>
#include <squirrel_manipulation_msgs/PushActionResult.h>

#include <squirrel_object_perception_msgs/StartObjectTracking.h>
#include <squirrel_object_perception_msgs/StopObjectTracking.h>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <squirrel_object_manipulation/RobotinoControl.hpp>
#include <squirrel_object_manipulation/conversion_utils.hpp>

#define PUSH_NAME "push"

typedef actionlib::SimpleActionServer<squirrel_manipulation_msgs::PushAction> pushServer;

class PushAction {

private:


    RobotinoControl *robotino;

    double controller_frequency_, tilt_nav_, tilt_perception_;

    std::string robot_base_frame_, global_frame_;

    geometry_msgs::PoseStamped push_goal_;
    std::string object_id_;

    //navigation path
    nav_msgs::Path pushing_path_;
    bool getPushPath();

    //robot pose update
    std::string pose_topic_;
    geometry_msgs::Pose2D pose_robot_;
    ros::Subscriber pose_sub_;
    void updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& );

    // push planning
    bool runPushPlan_;
    boost::mutex plan_push_mutex_;
    boost::condition_variable plan_push_cond_;
    boost::thread* plan_push_thread_;
    void planPushThread();

    //object tracking
    geometry_msgs::PoseStamped pose_object_;
    tf::TransformListener tf_listener_;
    bool trackingStart_;
    bool objectLost_;
    boost::thread* object_tracking_thread_;
    bool startTracking();
    bool stopTracking();
    void objectTrackingThread();

    //execute cycle

    bool executeCycle();

    void abortPush();





protected:

    ros::NodeHandle nh, private_nh;

    actionlib::SimpleActionServer<squirrel_manipulation_msgs::PushAction> pushServer;

    squirrel_manipulation_msgs::PushFeedback pushFeedback;

    squirrel_manipulation_msgs::PushResult pushResult;

public:

    PushAction(const std::string pushServerActionName);
    ~PushAction();

    void executePush(const squirrel_manipulation_msgs::PushGoalConstPtr &goal);



};

#endif
