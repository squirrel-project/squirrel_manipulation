#ifndef SQUIRREL_OBJECT_MANIPULATION_PUSH_ACTION_H_
#define SQUIRREL_OBJECT_MANIPULATION_PUSH_ACTION_H_

#include <ros/ros.h>
#include <iostream>

#include <std_msgs/Float64.h>
#include <boost/assert.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/range/numeric.hpp>

#include <tf/transform_listener.h>

#include <squirrel_rgbd_mapping_msgs/GetPushingPlan.h>
#include <squirrel_manipulation_msgs/PushAction.h>
#include <squirrel_manipulation_msgs/PushActionFeedback.h>
#include <squirrel_manipulation_msgs/PushActionGoal.h>
#include <squirrel_manipulation_msgs/PushActionResult.h>

#include <squirrel_object_perception_msgs/StartObjectTracking.h>
#include <squirrel_object_perception_msgs/StopObjectTracking.h>
#include <squirrel_object_perception_msgs/SceneObject.h>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <squirrel_object_manipulation/RobotinoControl.hpp>
#include <squirrel_object_manipulation/conversion_utils.hpp>

#include "../src/pushing/include/PushPlanner.hpp"
#include "../src/pushing/include/SimplePathFollowing.hpp"
#include "../src/pushing/include/SimplePush.hpp"
#include "../src/pushing/include/BangBangPush.hpp"
#include "../src/pushing/include/PIDPush.hpp"
#include "../src/pushing/include/PIDSimplePush.hpp"
#include "../src/pushing/include/PIDObjectPush.hpp"
#include "../src/pushing/include/DipoleField.hpp"
#include "../src/pushing/include/CentroidAlignment.hpp"
#include "../src/pushing/include/DynamicPush.hpp"

#include "mongodb_store/message_store.h"

#define PUSH_NAME "push"

typedef actionlib::SimpleActionServer<squirrel_manipulation_msgs::PushAction> pushServer;

class PushAction {

private:


    boost::shared_ptr<RobotinoControl> robotino;
    boost::shared_ptr<PushPlanner> push_planner_;

    tf::TransformListener tfl_;
    std::string node_name_;

    double controller_frequency_, tilt_nav_, tilt_perception_, pan_perception_, lookahead_, goal_toll_, object_diameter_, robot_diameter_, corridor_width_ ;

    std::string robot_base_frame_, global_frame_;

    bool state_machine_, clearance_nav_;
    bool nav_, artag_, firstSet, save_data_, sim_;
    double artag_offsetX, artag_offsetY, tag_t_prev;

    geometry_msgs::PoseStamped push_goal_;
    std::string object_id_;



   // mongodb_store::MessageStoreProxy message_store;

    //navigation path
    nav_msgs::Path pushing_path_;
    bool getPushPath();
    std::string octomap_topic_;
    std::string costmap_topic_;
    ros::Publisher octomap_pub_;
    ros::Publisher costmap_pub_;

    //robot pose update
    std::string pose_topic_;
    geometry_msgs::Pose2D pose_robot_;
    ros::Subscriber pose_sub_, marker_sub_ ;
    boost::mutex robot_pose_mutex_;
    void updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& );

    // push planning
    bool runPushPlan_;

    //object tracking
    double Olx, Oly;
    std::string tracker_tf_;
    geometry_msgs::TransformStamped t_artag;

    geometry_msgs::PoseStamped pose_object_;
    tf::TransformListener tf_listener_;
    bool trackingStart_;
    bool objectLost_;
    bool first_pose_;
    boost::thread* object_tracking_thread_;
    boost::mutex object_pose_mutex_;
    bool startTracking();
    bool stopTracking();
    bool getFirstObjectPose();
    void objectTrackingThread();

    void abortPush();
    void finishPush();
    void finishSuccess();

    //demo
    int demo_path;


protected:

    ros::NodeHandle nh, private_nh;

    actionlib::SimpleActionServer<squirrel_manipulation_msgs::PushAction> pushServer;

    squirrel_manipulation_msgs::PushFeedback pushFeedback;

    squirrel_manipulation_msgs::PushResult pushResult;

public:

    PushAction(const std::string pushServerActionName);
    ~PushAction();

    void executePush(const squirrel_manipulation_msgs::PushGoalConstPtr &goal);
    void goalCB();
    void preemptCB();
    void arCallback(tf::tfMessage msg);


};

#endif
