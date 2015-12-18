#include <squirrel_object_manipulation/pushing.hpp>


using namespace std;


PushAction::PushAction(const std::string std_PushServerActionName) :
    pushServer(nh, std_PushServerActionName, boost::bind(&PushAction::executePush, this, _1), false),
    private_nh("~"),
    runPushPlan_(false),
    trackingStart_(false),
    objectLost_(false),
    first_pose_(false)
{
    node_name_ = ros::this_node::getName();

    private_nh.param<std::string>("pose_topic", pose_topic_,"/squirrel_localizer_pose");
    private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_frame", global_frame_, std::string("map"));
    private_nh.param("controller_frequency", controller_frequency_, 10.0);
    private_nh.param("tilt_nav", tilt_nav_, 0.60);
    private_nh.param("tilt_perception", tilt_perception_, 0.60);
    private_nh.param("lookahead", lookahead_, 0.15);
    // private_nh.param("push_planner", push_planner_, new PushPlanner());
    push_planner_ = boost::shared_ptr<PushPlanner>(new SimplePathFollowing());

    pose_sub_ = nh.subscribe(pose_topic_, 2, &PushAction::updatePose, this);
    robotino = boost::shared_ptr<RobotinoControl>(new RobotinoControl(nh));

    object_tracking_thread_ = new boost::thread(boost::bind(&PushAction::objectTrackingThread, this));

    pushServer.start();
    ROS_INFO("(Push) Ready to push objects");
    cout << endl;
}

PushAction::~PushAction() {

    object_tracking_thread_->interrupt();
    object_tracking_thread_->join();

    delete object_tracking_thread_;
}

void PushAction::executePush(const squirrel_manipulation_msgs::PushGoalConstPtr &goal) {

    ROS_INFO("(Push) started push up of %s for manipulation \n",  goal->object_id.c_str());
    cout << endl;


    // initilize push
    runPushPlan_ = false;
    trackingStart_ = false;
    objectLost_ = false;

    //set controller rate
    ros::Rate lRate(controller_frequency_);

    //get goal
    push_goal_.pose = goal->pose;
    object_id_ = goal->object_id;

    if(!isQuaternionValid(goal->pose.orientation)){
        ROS_INFO("(Push): Invalid target orientation \n");
        cout << endl;
        abortPush();
        return;
    }

    ros::spinOnce();

    // start object tracking
    // move camera for vision
    robotino->moveTilt(tilt_nav_);
    if(startTracking()){
        ROS_INFO("(Push) Started tracking of the %s \n",  goal->object_id.c_str());
        trackingStart_ = true;
    }
    else{
        ROS_ERROR("Push: Start tracking of the %s failed \n",  goal->object_id.c_str());
        abortPush();
        return;
    }

    cout << endl;

    if(getFirstObjectPose()){
        ROS_INFO("(Push) Got first pose \n",  goal->object_id.c_str());
    }
    else{
        ROS_ERROR("Push: Getting first pose failed \n" ,  goal->object_id.c_str());
        abortPush();
        return;
    }
    cout << endl;

    //getting path from navigation
    if(!getPushPath()){
        ROS_ERROR("Push: Getting a path from navigation failed \n");
        abortPush();
        return;
    }
    cout << endl;

    //initialize push planner
    push_planner_->initialize(robot_base_frame_, global_frame_, pose_robot_, pose_object_, pushing_path_, lookahead_);
    push_planner_->visualisationOn();


    //main push loop
    while (nh.ok() &&  !push_planner_->goal_reached_){

        push_planner_->updatePushPlanner(pose_robot_,pose_object_);
        geometry_msgs::Twist cmd = push_planner_->getVelocities();
       // cout<<"move"<<endl<<cmd <<endl;
        robotino->singleMove(cmd.linear.x,0,0.0,0.0,0.0,cmd.angular.z);

        lRate.sleep();
    }

    if (push_planner_->goal_reached_){
        ROS_INFO("Goal reached sucessfully \n");
        finishSuccess();
        return;
    }

    //end of action instance

    //if pushing did not result with success for any reason
    abortPush();
    return;


}


void PushAction::objectTrackingThread(){

    ros::Rate lRate(controller_frequency_);
    ros::NodeHandle n;

    tf::StampedTransform trans;
    first_pose_ = false;

    // boost::unique_lock<boost::mutex> lock(object_pose_mutex_);

    while (n.ok()){
        // lock.lock();
        lRate.sleep();

        if(trackingStart_&&(first_pose_)){
            try {
                tf_listener_.waitForTransform(global_frame_, object_id_, ros::Time::now(), ros::Duration(0.2));
                tf_listener_.lookupTransform(global_frame_, object_id_, ros::Time(0), trans);
                pose_object_ = tf_stamped2pose_stamped(trans);
            } catch (tf::TransformException& ex) {
                std::string ns = ros::this_node::getNamespace();
                std::string node_name = ros::this_node::getName();
                ROS_ERROR("Push: %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
                abortPush();
            }
        }
        // lock.unlock();
    }
}


void PushAction::updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg )
{
    pose_robot_.x = pose_msg->pose.pose.position.x;
    pose_robot_.y = pose_msg->pose.pose.position.y;
    pose_robot_.theta = tf::getYaw(pose_msg->pose.pose.orientation);
}

bool PushAction::getFirstObjectPose(){

    first_pose_ = false;
    tf::StampedTransform trans;

    if(trackingStart_&&(!first_pose_)){
        try {
            tf_listener_.waitForTransform(global_frame_, object_id_, ros::Time::now(), ros::Duration(10.0));
            tf_listener_.lookupTransform(global_frame_, object_id_, ros::Time(0), trans);
            pose_object_ = tf_stamped2pose_stamped(trans);
            first_pose_ = true;
        } catch (tf::TransformException& ex) {
            std::string ns = ros::this_node::getNamespace();
            std::string node_name = ros::this_node::getName();
            ROS_ERROR("Push: %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
        }

    }

    return first_pose_;

}

bool PushAction::getPushPath(){

    squirrel_rgbd_mapping_msgs::GetPushingPlan srvPlan;

    // part which has to be revised
    geometry_msgs::PoseStamped start_m;
    try {
        tfl_.waitForTransform(global_frame_, "/base_link", ros::Time::now(), ros::Duration(1.0));
        tfl_.transformPose("/base_link", pose_object_, start_m);
        start_m.header.frame_id = "/base_link";
    } catch ( tf::TransformException& ex ) {
        ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
        return true;
    }

    // Getting pushing plan
    srvPlan.request.start.x =  start_m.pose.position.x;
    srvPlan.request.start.y =  start_m.pose.position.y;
    srvPlan.request.start.theta =  0;

    srvPlan.request.goal.x = push_goal_.pose.position.x;
    srvPlan.request.goal.y = push_goal_.pose.position.y;
    srvPlan.request.goal.theta = tf::getYaw(push_goal_.pose.orientation);

    // To be replaced with the the object size !!!

    geometry_msgs::Point32 p1, p2, p3, p4;

    p1.x = 0.20; p1.y = -0.20;
    p2.x = 0.20; p1.y = 0.20;
    p3.x = -0.20; p1.y = -0.20;
    p4.x = -0.20; p1.y = 0.20;

    //----------------------------------------

    srvPlan.request.object.points.push_back(p1);
    srvPlan.request.object.points.push_back(p2);
    srvPlan.request.object.points.push_back(p3);
    srvPlan.request.object.points.push_back(p4);

    if ( ros::service::call("/getPushingPlan", srvPlan) ) {
        if ( srvPlan.response.plan.poses.empty() ) {
            ROS_WARN("Push:got an empty plan");
        } else {
            BOOST_ASSERT_MSG( srvPlan.response.plan.header.frame_id == "/odom" ||
                              srvPlan.response.plan.header.frame_id == "odom" ,
                              "returned path is not in requested frame");
            ROS_INFO("Push: got a path for pushing");
        }
    } else {
        ROS_ERROR("Push: unable to communicate with /getPushingPlan");
        return false;
    }

    // Convert plan to global_frame_
    pushing_path_.header.frame_id = global_frame_;
    for (unsigned int i=0; i<srvPlan.response.plan.poses.size(); ++i) {
        try {
            geometry_msgs::PoseStamped p;
            tfl_.waitForTransform("/odom", global_frame_, ros::Time::now(), ros::Duration(0.5));
            tfl_.transformPose(global_frame_, srvPlan.response.plan.poses[i], p);
            p.header.frame_id = global_frame_;
            pushing_path_.poses.push_back(p);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
            return false;
        }
    }

    ROS_INFO("Push: path ready for pushing \n");

    return true;

}

void PushAction::abortPush(){

    ROS_INFO("(Push) Aborting push action \n");

    pushResult.result_status = "failure";
    pushServer.setAborted(pushResult);
    finishPush();

}

void PushAction::finishPush(){

    if(trackingStart_){
        if (stopTracking()){
            ROS_INFO("(Push) Object tracking stopped \n");
        }
        else{
            ROS_ERROR("(Push) Object tracking did not finish properly \n");
        }
    }

    trackingStart_ = false;

    //moving tilt for navigation configuration
    robotino->moveTilt(tilt_nav_);
    ros::spinOnce();

}
void PushAction::finishSuccess(){

    ROS_INFO("(Push) Push action  executed sucessfully \n");

    pushResult.result_status = "success";
    pushServer.setSucceeded(pushResult);
    finishPush();

}

bool PushAction::startTracking() {
    squirrel_object_perception_msgs::StartObjectTracking srvStartTrack;
    srvStartTrack.request.object_id.data = object_id_;
    return(ros::service::call("/squirrel_start_object_tracking", srvStartTrack));
}

bool PushAction::stopTracking() {
    squirrel_object_perception_msgs::StopObjectTracking srvStopTrack;
    bool stopped = false;
    int count = 0;
    while((count <10) || (!stopped)) {
        if(ros::service::call("/squirrel_stop_object_tracking", srvStopTrack)) stopped = true;
        count++;
    }
    if (stopped){
        trackingStart_ = false;
        first_pose_ = false;
        runPushPlan_ = false;
        objectLost_ = false;
        ROS_INFO("(Push) tracking have finished sucessfully");
        return true;
    }
    ROS_INFO("(Push) tracking have not finished sucessfully");
    return false;

}

int main(int argc, char** argv) {


    ros::init(argc, argv, "base_pushing");

    PushAction push(PUSH_NAME);
    ros::spin();

    return 0;

}
