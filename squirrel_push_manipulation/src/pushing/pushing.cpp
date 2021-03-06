#include <squirrel_push_manipulation/pushing.hpp>

#include <nav_msgs/Odometry.h>
#include <robotino_msgs/ResetOdometry.h>


using namespace std;
using namespace arma;


PushAction::PushAction(const std::string std_PushServerActionName) :
    pushServer(nh, std_PushServerActionName , boost::bind(&PushAction::executePush, this, _1), false),
    private_nh("~"),
    runPushPlan_(false),
    trackingStart_(false),
    objectLost_(false),
    first_pose_(false),
    firstSet (false)
{
    node_name_ = ros::this_node::getName();

    private_nh.param("pose_topic", pose_topic_,std::string("/squirrel_2d_localizer/pose"));
    private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("push_action_active", action_active_topic_, std::string("/pushing_action"));
    private_nh.param("global_frame", global_frame_, std::string("/map"));
    private_nh.param("controller_frequency", controller_frequency_, 20.00);
    private_nh.param("tilt_nav", tilt_nav_, -0.3);
    private_nh.param("tilt_perception", tilt_perception_, 0.3); //used as coordinates in robot frame
    private_nh.param("pan_perception", pan_perception_, 0.0);

    private_nh.param("goal_tolerance", goal_toll_, 0.1);
    private_nh.param("state_machine", state_machine_, false);
    private_nh.param("object_diameter", object_diameter_, 0.1);
    private_nh.param("robot_diameter", robot_diameter_, 0.46);
    private_nh.param("corridor_width", corridor_width_ , 1.6);


    private_nh.param("clearance_nav", clearance_nav_, true);
    private_nh.param("check_collisions", check_collisions_, false);
    private_nh.param("sim_", sim_, true);
    private_nh.param("save_data", save_data_, false);
    private_nh.param("lookahead", lookahead_, 0.10);
    private_nh.param("relaxation_", relaxation_, true);



    push_planner_ = boost::shared_ptr<PushPlanner>(new DynamicPush());



    //set callback for cancel request

    pushServer.registerPreemptCallback(boost::bind(&PushAction::preemptCB, this));

    pose_sub_ = nh.subscribe(pose_topic_, 2, &PushAction::updatePose, this);
    active_pub_ = nh.advertise<std_msgs::Bool>(action_active_topic_, 100);
    robotino = boost::shared_ptr<RobotinoControl>(new RobotinoControl(nh));

    object_tracking_thread_ = new boost::thread(boost::bind(&PushAction::objectTrackingThread, this));
    irsensors_thread_ = new boost::thread(boost::bind(&PushAction::checkCollisionsThread, this));

    pushServer.start();
    ROS_INFO("(Push) Ready to push objects");
    cout << endl;
}

PushAction::~PushAction() {

    object_tracking_thread_->interrupt();
    object_tracking_thread_->join();
    robotino->stopRobot();

    delete object_tracking_thread_;
}

void PushAction::executePush(const squirrel_manipulation_msgs::PushGoalConstPtr &goal) {

    ROS_INFO("(Push) Started pushing of %s  \n",  goal->object_id.c_str());
    cout << endl;

    try{

        if(!nav_ && !sim_){
            ros::ServiceClient rO=nh.serviceClient<robotino_msgs::ResetOdometry>("/reset_odometry");
            robotino_msgs::ResetOdometry R;
            R.request.x=0;
            R.request.y=0;
            R.request.phi=0;

            if( rO.call(R))
            {
                ROS_INFO("(Push) odometry RESET successsful \n");
                cout<<endl;
            }
            else{
                ROS_ERROR("(Push) odometry RESET successsful \n");
                cout<<endl;
            }
            sleep(1.0);
        }



        ROS_INFO("(Push) Start Initialization \n");
        cout << endl;

        // initilize push
        runPushPlan_ = true;
        trackingStart_ = false;
        objectLost_ = false;
        obstacles_ = false;

        //set controller rate
        ros::Rate lRate(controller_frequency_);

        //get goal
        push_goal_.pose = goal->pose;
        object_id_ = goal->object_id;



        if(!artag_ && !sim_ ){
            //get the object diameter
            mongodb_store::MessageStoreProxy message_store(nh);

            //get the object diameter

            // fetch position of object from message store
            std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;


            if(message_store.queryNamed<squirrel_object_perception_msgs::SceneObject>(object_id_, results)) {
                //cout << "matching objects to '" << object_id_ << "'\n";
                for(size_t i = 0; i < results.size(); i++)
                    // cout << "  " << results[i]->bounding_cylinder.diameter << "\n";
                    if(results.size()<1) {
                        ROS_ERROR(" no matching obID %s", object_id_.c_str());
                        return;
                    }
                if(results.size()>1)
                    ROS_ERROR("(Push)  multiple objects share the same wpID");
            } else {
                ROS_ERROR("(Push) could not query message store to fetch object size");
                return;
            }

            object_diameter_ = results[0]->bounding_cylinder.diameter;
            if (object_diameter_ < 0.05) object_diameter_ = 0.20;
            if (object_diameter_ > 1.00) {
                ROS_ERROR("(Push) Invalid value for the object size \n");
                abortPush();
            }
        }
        else if(artag_) firstSet = false;


        if(!isQuaternionValid(goal->pose.orientation)){
            ROS_INFO("(Push): Invalid target orientation \n");
            cout << endl;
            abortPush();
            return;
        }

        ros::spinOnce();

        dynamic_reconfigure::Config costmap_conf;

        costmap_param.name = "use_kinect";
        costmap_param.value = false;
        costmap_conf.bools.push_back(costmap_param);

        costmap_param.name = "use_laser_scan";
        costmap_param.value = false;
        costmap_conf.bools.push_back(costmap_param);

        costmap_srv_req.config = costmap_conf;

        if ( ros::service::call("/move_base/global_costmap/ObstaclesLayer/set_parameters", costmap_srv_req, costmap_srv_resp)) {
        } else {
            ROS_ERROR("(Push) unable to communicate with /move_base/global_costmap/ObstaclesLayer/set_parameters");
            abortPush();
            return ;
        }

        ros::spinOnce();
        ROS_INFO("(Push) Kinect and laser layer off \n");


        // move camera for vision
        squirrel_view_controller_msgs::LookAtPosition srvLookAt;
        srvLookAt.request.target.header.frame_id = "base_link";
        srvLookAt.request.target.pose.position.x = tilt_perception_;
        srvLookAt.request.target.pose.position.y = pan_perception_;
        srvLookAt.request.target.pose.position.z = 0.0;
        srvLookAt.request.target.pose.orientation.x = 0.0;
        srvLookAt.request.target.pose.orientation.y = 0.0;
        srvLookAt.request.target.pose.orientation.z = 0.0;
        srvLookAt.request.target.pose.orientation.w = 1.0;

        if (ros::service::call("/squirrel_view_controller/look_at_position", srvLookAt)) {
            ROS_INFO("(Push) Camera moved for tracking \n");
        }
        else {
            ROS_ERROR("(Push) unable to communicate with /squirrel_view_controller/look_at_position");
            abortPush();
            return;
        }

        //robotino->moveTilt(tilt_perception_);
        ros::spinOnce();

        ROS_INFO("(Push) Waiting for the tracker of the %s to start \n", goal->object_id.c_str());
        if(startTracking()){
            trackingStart_ = true;
        }
        else{
            ROS_ERROR("(Push) Start tracking of the %s failed \n", goal->object_id.c_str());
            abortPush();
            return;
        }
        cout << endl;

        if(getFirstObjectPose()){
            ROS_INFO("(Push) Tracking started. \n");
        }
        else{
            ROS_ERROR("(Push) Getting first pose failed \n");
            abortPush();
            return;
        }
        cout << endl;

        //getting path from navigation
        if(!getPushPath()){
            ROS_ERROR("(Push) Getting a path from navigation failed \n");
            abortPush();
            return;
        }
        cout << endl;


        //initialize push planner
        if (runPushPlan_){
            push_planner_->initialize(robot_base_frame_, global_frame_, pose_robot_, pose_object_, pushing_path_, lookahead_, goal_toll_, state_machine_, controller_frequency_, object_diameter_, robot_diameter_, corridor_width_, corridor_width_array_, fixed_, relaxation_);
            push_planner_->visualisationOn();
            if (this->sim_) push_planner_->setSim();
            push_planner_->startPush();
        }

        double secs =ros::Time::now().toSec();

        try{
            //main push loop
            while (nh.ok() &&  push_planner_->push_active_  && runPushPlan_ ){
                push_planner_->updatePushPlanner(pose_robot_, pose_object_);
                geometry_msgs::Twist cmd = push_planner_->getControlCommand();
                //cout<<"execute cmd "<<cmd<<endl;
                robotino->singleMove(cmd.linear.x, cmd.linear.y,0.0,0.0,0.0,cmd.angular.z);

                lRate.sleep();

            }
        }
        catch (...){
        }
        push_planner_->setExperimentName(object_id_);
        if (save_data_) push_planner_->saveData("path");

        if(obstacles_){
            ROS_INFO("(Push) Obstacle detected");
            cout << endl;
        }

        if (push_planner_->goal_reached_){
            ROS_INFO("Goal reached sucessfully \n");
            finishSuccess();
            return;
        }else{
            //if pushing did not result with success for any reason
            abortPush();
        }
        double secs2 = ros::Time::now().toSec();
        cout<<"(Push)push duration "<<secs2 - secs;

        cout << endl;

        //end of action instance
    }
    catch(...){
        ROS_ERROR("Exception occurred\n");
        abortPush();
    }


    return;


}

bool PushAction::getFirstObjectPose(){

    first_pose_ = false;
    tf::StampedTransform trans;

    if(!artag_){

        if(trackingStart_&&(!first_pose_)){
            try {
                tf_listener_.waitForTransform(global_frame_, object_id_, ros::Time::now(), ros::Duration(10.0));
                tf_listener_.lookupTransform(global_frame_, object_id_, ros::Time(0), trans);
                pose_object_ = tf_stamped2pose_stamped(trans);
                first_pose_ = true;
            } catch (tf::TransformException& ex) {
                std::string ns = ros::this_node::getNamespace();
                std::string node_name = ros::this_node::getName();
                ROS_ERROR("(Push) %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
            }

        }

        return first_pose_;
    }
    else return true;

}

void PushAction::checkCollisionsThread(){

    ros::Rate lRate(controller_frequency_);
    ros::NodeHandle n;

    while (n.ok() && check_collisions_){

        if(robotino->checkDistancesPush(0.05)){

            push_planner_->push_active_ = false;
            obstacles_ = true;

        }
        lRate.sleep();
    }

}

void PushAction::objectTrackingThread(){

    ros::Rate lRate(controller_frequency_);
    ros::NodeHandle n;

    tf::StampedTransform trans;
    first_pose_ = false;

    while (n.ok()){


        if(!nav_){

            robot_pose_mutex_.lock();
            nav_msgs::Odometry Odometry = robotino->getOdom();
            //robotino position in /odom world
            pose_robot_.x = Odometry.pose.pose.position.x;
            pose_robot_.y = Odometry.pose.pose.position.y;
            pose_robot_.theta = tf::getYaw(Odometry.pose.pose.orientation);
            robot_pose_mutex_.unlock();
        }

        object_pose_mutex_.lock();
        lRate.sleep();

        if(!artag_){

            if(trackingStart_&&first_pose_){
                try {
                    tf_listener_.waitForTransform(global_frame_, object_id_, ros::Time::now(), ros::Duration(0.2));
                    tf_listener_.lookupTransform(global_frame_, object_id_, ros::Time(0), trans);
                    pose_object_ = tf_stamped2pose_stamped(trans);
                } catch (tf::TransformException& ex) {
                    std::string ns = ros::this_node::getNamespace();
                    std::string node_name = ros::this_node::getName();
                    ROS_ERROR("(Push) Tracking error %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
                    abortPush();
                }
            }
        }

        if(artag_&firstSet){

            pose_object_.header.frame_id =  global_frame_;


            if (t_artag.header.stamp.nsec != tag_t_prev){

                double Olxt = t_artag.transform.translation.x  - artag_offsetX;
                double Olyt = t_artag.transform.translation.y  - artag_offsetY;
                if(distancePoints(Olx, Oly, Olxt, Olyt)< 0.05){
                    Olx = Olxt;
                    Oly = Olyt;
                }
                double th = pose_robot_.theta;

                pose_object_.pose.position.x = pose_robot_.x - Oly*cos(th)+Olx*sin(th);;
                pose_object_.pose.position.y = pose_robot_.y - Oly*sin(th)-Olx*cos(th);
                pose_object_.pose.position.z = 0;
                pose_object_.pose.orientation = t_artag.transform.rotation;
                tag_t_prev = t_artag.header.stamp.nsec;
            }

        }
        object_pose_mutex_.unlock();
    }
}


void PushAction::updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg ){

    if(nav_){

        robot_pose_mutex_.lock();
        pose_robot_.x = pose_msg->pose.pose.position.x;
        pose_robot_.y = pose_msg->pose.pose.position.y;
        pose_robot_.theta = tf::getYaw(pose_msg->pose.pose.orientation);
        robot_pose_mutex_.unlock();
    }
}


bool PushAction::getPushPath(){

    try{


        //clear costmap
        squirrel_navigation_msgs::ClearCostmapRegion srvPlan;

        geometry_msgs::PoseStamped start_m;
        try {
            tfl_.waitForTransform(global_frame_, "/map", ros::Time::now(), ros::Duration(1.0));
            tfl_.transformPose("/map", pose_object_, start_m);
            start_m.header.frame_id = "/map";
        } catch ( tf::TransformException& ex ) {
            ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
            return true;
        }


        // Set object polygon
        geometry_msgs::Point32 p1, p2, p3, p4;
        double d = 0.25;

        p1.x = start_m.pose.position.x + d; p1.y = start_m.pose.position.y - d;
        p2.x = start_m.pose.position.x + d; p2.y = start_m.pose.position.y + d;
        p3.x = start_m.pose.position.x - d; p3.y = start_m.pose.position.y - d;
        p4.x = start_m.pose.position.x - d; p4.y = start_m.pose.position.y + d;

        // Make request

        srvPlan.request.region.points.push_back(p1);
        srvPlan.request.region.points.push_back(p2);
        srvPlan.request.region.points.push_back(p3);
        srvPlan.request.region.points.push_back(p4);
        srvPlan.request.sleep = ros::Duration(0.1);


        if ( ros::service::call("/move_base/global_costmap/ObstaclesLayer/clearCostmapRegion", srvPlan) ) {
        } else {
            ROS_ERROR("(Push) unable to communicate with move_base/global_costmap/ObstaclesLayer/clearCostmapRegion");
            return false;
        }
        ROS_INFO("(Push) Object cleared from the costmap \n");

        //get plan
        nav_msgs::GetPlan getPlanSrv;

        // Getting pushing plan
        getPlanSrv.request.start =  start_m;
        getPlanSrv.request.start.header.frame_id = global_frame_;
        getPlanSrv.request.start.header.stamp = ros::Time::now();
        getPlanSrv.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(pose_robot_.theta);
        getPlanSrv.request.goal = push_goal_;
        getPlanSrv.request.goal.header.frame_id = global_frame_;
        getPlanSrv.request.tolerance = 0.0;
        getPlanSrv.request.goal.header.stamp = ros::Time::now();

        if (ros::service::call("/move_base/GlobalPlanner/Dijkstra/make_plan", getPlanSrv)) {
            if (getPlanSrv.response.plan.poses.empty() ) {
                ROS_WARN("(Push) Got an empty plan");
                return false;
            }
        }
        else {
            ROS_ERROR("(Push) unable to communicate with /move_base/GlobalPlanner/Dijkstra/make_plan");
            return false;
        }


        ROS_INFO("(Push) Got pushing plan \n");
        pushing_path_.poses.clear();
        pushing_path_.header.frame_id = global_frame_;
        for (unsigned int i = 0; i<getPlanSrv.response.plan.poses.size(); ++i) {

            geometry_msgs::PoseStamped p = getPlanSrv.response.plan.poses[i];
            p.header.frame_id = global_frame_;
            pushing_path_.poses.push_back(p);

        }


        //Get clearance from navigation
        if(clearance_nav_){
            corridor_width_ = -1.0;

            squirrel_navigation_msgs::GetPathClearance ClearSrv;

            ClearSrv.request.plan = getPlanSrv.response.plan;


            if (ros::service::call("/move_base/global_costmap/ObstaclesLayer/getPathClearance", ClearSrv) ) {
                if ( ClearSrv.response.proximities.empty() ) {
                    ROS_WARN("(Push) Got an empty clearance");
                    return false;
                }
            } else {
                ROS_ERROR("(Push) unable to communicate with /move_base/global_costmap/ObstaclesLayer/getPathClearance");
                return false;
            }
            ROS_INFO("(Push) Got clearance \n");

            corridor_width_array_ .clear();
            for (int i = 0; i < ClearSrv.response.proximities.size(); i++ ){
                //cout<<ClearSrv.response.proximities.at(i)<<endl;
                double d = ClearSrv.response.proximities.at(i);
                if (d < robot_diameter_ + object_diameter_) {d = robot_diameter_ + object_diameter_; cout << "small d"<<endl;}
                if (d > 20 * robot_diameter_) {d = 20 * robot_diameter_; cout << "large d"<<endl;}
                corridor_width_array_ .push_back(d);

            }

        }

        std_msgs::Bool active_msg_;
        active_msg_.data = false;
        active_pub_.publish(active_msg_);

        ROS_INFO("(Push) Path ready for pushing \n");
        return true;
    }
    catch (...){
        ROS_ERROR("(Push) Path getting failed \n");
        return false;
    }

}


void PushAction::abortPush(){

    ROS_INFO("(Push) Aborting push action \n");

    finishPush();
    pushResult.result_status = "failure";
    pushServer.setAborted(pushResult);
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
    obstacles_ = false;
    runPushPlan_ = false;
    pushing_path_.poses.clear();
    corridor_width_array_.clear();
    if(nav_) push_planner_->deleteMarkers();

    //moving tilt for navigation configuration
    //robotino->moveTilt(tilt_nav_);
    //ros::spinOnce();
    //ROS_INFO("(Push) Camera in the pose for navigation \n");

    dynamic_reconfigure::Config costmap_conf;

    costmap_param.name = "use_kinect";
    costmap_param.value = true;
    costmap_conf.bools.push_back(costmap_param);

    costmap_param.name = "use_laser_scan";
    costmap_param.value = true;
    costmap_conf.bools.push_back(costmap_param);


    costmap_srv_req.config = costmap_conf;

    if ( ros::service::call("/move_base/global_costmap/ObstaclesLayer/set_parameters", costmap_srv_req, costmap_srv_resp)) {
    } else {
        ROS_ERROR("(Push) unable to communicate with /move_base/global_costmap/ObstaclesLayer/set_parameters");
        abortPush();
        return ;
    }

    //turn on costmap

    ros::spinOnce();

    ROS_INFO("(Push) Kinect and laser  on \n");

}
void PushAction::finishSuccess(){

    finishPush();

    ROS_INFO("(Push) Push action executed sucessfully \n");

    pushResult.result_status = "success";
    pushServer.setSucceeded(pushResult);


}

bool PushAction::startTracking() {

    squirrel_object_perception_msgs::StartObjectTracking srvStartTrack;
    srvStartTrack.request.object_id.data = object_id_;
    if(!sim_) return(ros::service::call("/squirrel_start_lump_tracking", srvStartTrack));
    else return(ros::service::call("/squirrel_start_object_tracking", srvStartTrack));

}

bool PushAction::stopTracking() {
    bool track = true;
    squirrel_object_perception_msgs::StopObjectTracking srvStopTrack;
    if(!sim_) track = ros::service::call("/squirrel_stop_lump_tracking", srvStopTrack);
    else track = ros::service::call("/squirrel_stop_object_tracking", srvStopTrack);
    trackingStart_ = false;
    first_pose_ = false;
    objectLost_ = false;
    firstSet = false;

    return track;
}


void PushAction::preemptCB(){

    ROS_INFO("(Push) Canceled push action by the high-level planner");
    cout<<endl;
    runPushPlan_ = false;
    //this->finishPush();
    //pushServer.setPreempted();

}


int main(int argc, char** argv) {

    ros::init(argc, argv, "push_manipulation");

    PushAction push(PUSH_NAME);
    ros::spin();

    return 0;

}

