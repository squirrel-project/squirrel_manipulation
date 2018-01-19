#include <squirrel_push_manipulation/smash.hpp>


using namespace std;
using namespace arma;

enum SmashState {

    RELOCATE,
    SMASH,
    GO_HOME,
    STOP
};

SmashAction::SmashAction(const std::string std_SmashServerActionName) :
    smashServer(nh, std_SmashServerActionName, boost::bind(&SmashAction::executeSmash, this, _1), false),
    private_nh("~")
{
    smashServer.start();
    ROS_INFO("Smash: server started \n");

    private_nh.param<std::string>("smash/pose_topic", pose_topic_,"/squirrel_localizer_pose");
    private_nh.param("smash/robot_diameter", robot_diameter_, 0.23);
    private_nh.param("smash/max_distance", max_distance_, 1.5);
    private_nh.param("smash/end_toll", end_toll_, 0.06);
    private_nh.param("smash/angle_toll", angle_toll_, 0.2);
    private_nh.param("smash/attraction_coefficient", attraction_coefficient_, 1.6);
    private_nh.param("smash/rotation_coefficient", rotation_coefficient_, 0.5);
    private_nh.param("smash/velocity_angular_max", vel_ang_max_ , 0.3);
    private_nh.param("smash/velocity_linear_max", vel_lin_max_ , 0.3);
    private_nh.param("smash/velocity_linear_back_max", vel_lin_back_max_ , 0.2);
    private_nh.param("smash/shake", shake_ , false);

    pose_sub_ = nh.subscribe(pose_topic_, 2, &SmashAction::updatePose, this);
    marker_target_c_ = nh.advertise<visualization_msgs::Marker>("/smash_action/current_target", 10, true);

    robotino = new RobotinoControl(nh);
}



void SmashAction::executeSmash(const squirrel_manipulation_msgs::SmashGoalConstPtr &goal) {


    ROS_INFO("Smash: action started \n");
    cout<<endl;

    ros::Rate lRate(20.0);
    ros::spinOnce();

    double goalx, goaly;

    goalx = goal->pose.position.x;
    goaly = goal->pose.position.y;

    bool smash_success_ = false;

    publishMarker(goalx, goaly);

    if(distancePoints(pose_robot_.x, pose_robot_.y, goalx, goaly) > max_distance_ + robot_diameter_){

        ROS_ERROR(" Smash: Robot distance to the clutter larger than the allowed one. ");

        ros::spinOnce();
        robotino->stopRobot();
        smashResult.result_status = "failure";
        smashServer.setAborted(smashResult);

    }

    geometry_msgs::Pose2D pose_start_ = pose_robot_;
    geometry_msgs::Twist cmd = getNullTwist();

    //orientate first if clutter not in front of the robot
    double aR2G = getVectorAngle(goalx - pose_robot_.x, goaly - pose_robot_.y);
    while(abs(rotationDifference(aR2G, pose_robot_.theta)) > angle_toll_){
        cmd = getNullTwist();
        aR2G = getVectorAngle(goalx - pose_robot_.x, goaly - pose_robot_.y);
        cmd.angular.z = rotation_coefficient_ * rotationDifference(aR2G, pose_robot_.theta);
        if(fabs(cmd.linear.x) > vel_lin_max_) cmd.linear.x = (cmd.linear.x > 0 ? vel_lin_max_ : - vel_lin_max_);
        if(fabs(cmd.linear.y) > vel_lin_max_) cmd.linear.y = (cmd.linear.y > 0 ? vel_lin_max_ : - vel_lin_max_);
        if(fabs(cmd.angular.z) > vel_ang_max_) cmd.angular.z = (cmd.angular.z > 0 ? vel_ang_max_ : - vel_ang_max_);

        robotino->singleMove(cmd);
        lRate.sleep();
    }


    ROS_INFO(" Smash: Orientation done. Starting smash ");
    cout<<endl;

    // smash

    while(distancePoints(pose_robot_.x, pose_robot_.y, goalx, goaly) > end_toll_){
        cmd = getNullTwist();

        // Attraction
        double G_attr_x = -attraction_coefficient_ * (pose_robot_.x - goalx);
        double G_attr_y = -attraction_coefficient_ * (pose_robot_.y - goaly);

        vec vel_R_  = rotate2DVector(G_attr_x ,G_attr_y, -pose_robot_.theta);
        cmd.linear.x = vel_R_(0);
        cmd.linear.y = vel_R_(1);

        //orientation cmd
        aR2G = getVectorAngle(goalx - pose_robot_.x, goaly - pose_robot_.y);
        cmd.angular.z = rotation_coefficient_ * rotationDifference(aR2G, pose_robot_.theta);

        if(fabs(cmd.linear.x) > vel_lin_max_) cmd.linear.x = (cmd.linear.x > 0 ? vel_lin_max_ : - vel_lin_max_);
        if(fabs(cmd.linear.y) > vel_lin_max_) cmd.linear.y = (cmd.linear.y > 0 ? vel_lin_max_ : - vel_lin_max_);
        if(fabs(cmd.angular.z) > vel_ang_max_) cmd.angular.z = (cmd.angular.z > 0 ? vel_ang_max_ : - vel_ang_max_);

        robotino->singleMove(cmd);
        lRate.sleep();
    }

    ROS_INFO(" Smash: Smash done. Going to start position ");
    cout<<endl;

    if(distancePoints(pose_robot_.x, pose_robot_.y, goalx, goaly) <= end_toll_) smash_success_ = true;

    //shake
    if(shake_){

        geometry_msgs::Pose2D pose_end_ = pose_robot_;

        // go left
        while(distancePoints(pose_robot_.x, pose_robot_.y, pose_end_.x, pose_end_.y) < end_toll_){
            cmd = getNullTwist();
            cmd.linear.y = vel_lin_max_;
            robotino->singleMove(cmd);
            lRate.sleep();

        }
        ROS_INFO(" Smash: left 1 done ");
        cout<<endl;

        // go right
        while(distancePoints(pose_robot_.x, pose_robot_.y, pose_end_.x, pose_end_.y) < end_toll_*2){
            cmd = getNullTwist();
            cmd.linear.y = -vel_lin_max_;
            robotino->singleMove(cmd);
            lRate.sleep();

        }
        ROS_INFO(" Smash: right done ");
        cout<<endl;

        // go left
        while(distancePoints(pose_robot_.x, pose_robot_.y, goalx, goaly) > end_toll_){
            cmd = getNullTwist();
            // Attraction
            double G_attr_x = -attraction_coefficient_ * (pose_robot_.x - goalx);
            double G_attr_y = -attraction_coefficient_ * (pose_robot_.y - goaly);

            vec vel_R_  = rotate2DVector(G_attr_x ,G_attr_y, -pose_robot_.theta);
            cmd.linear.x = vel_R_(0);
            cmd.linear.y = vel_R_(1);
            if(fabs(cmd.linear.x) > vel_lin_max_) cmd.linear.x = (cmd.linear.x > 0 ? vel_lin_max_ : - vel_lin_max_);
            if(fabs(cmd.linear.y) > vel_lin_max_) cmd.linear.y = (cmd.linear.y > 0 ? vel_lin_max_ : - vel_lin_max_);
            robotino->singleMove(cmd);
            lRate.sleep();
        }

        ROS_INFO(" Smash: left 2 done ");
        cout<<endl;
    }


    // return to start
    while(distancePoints(pose_robot_.x, pose_robot_.y, pose_start_.x, pose_start_.y) > end_toll_){
        cmd = getNullTwist();

        // Attraction
        double G_attr_x = -attraction_coefficient_ * (pose_robot_.x - pose_start_.x);
        double G_attr_y = -attraction_coefficient_ * (pose_robot_.y - pose_start_.y);

        vec vel_R_  = rotate2DVector(G_attr_x ,G_attr_y, -pose_robot_.theta);
        cmd.linear.x = vel_R_(0);
        cmd.linear.y = vel_R_(1);


        if(fabs(cmd.linear.x) > vel_lin_back_max_) cmd.linear.x = (cmd.linear.x > 0 ? vel_lin_back_max_ : - vel_lin_back_max_);
        if(fabs(cmd.linear.y) > vel_lin_back_max_) cmd.linear.y = (cmd.linear.y > 0 ? vel_lin_back_max_ : - vel_lin_back_max_);

        robotino->singleMove(cmd);
        lRate.sleep();
    }
    ROS_INFO(" Smash: Robot back in start position ");
    cout<< endl;


    ros::spinOnce();
    robotino->stopRobot();
    ROS_INFO(" Smash: Robotino stop");
    cout<< endl;

    if(smash_success_){
        smashResult.result_status = "success";
        smashServer.setSucceeded(smashResult);
        ROS_INFO(" Smash: Sucessfuly finished smash ");
        cout<< endl;
    }
    else{
        smashResult.result_status = "failure";
        smashServer.setAborted(smashResult);
        ROS_INFO(" Smash: failed ");
        cout<< endl;}
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "manipulation");

    SmashAction smash(SMASH_NAME);
    ros::spin();

    return 0;

}

void SmashAction::updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg )
{
    pose_robot_.x = pose_msg->pose.pose.position.x;
    pose_robot_.y = pose_msg->pose.pose.position.y;
    pose_robot_.theta = tf::getYaw(pose_msg->pose.pose.orientation);
}


void SmashAction::publishMarker(double goalx, double goaly) {

    geometry_msgs::PoseStamped t_pose;
    t_pose.pose.position.x = goalx;
    t_pose.pose.position.y = goaly;
    t_pose.pose.position.z = 0.0;
    t_pose.pose.orientation.w = 1.0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "smash_action";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = t_pose.pose;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_target_c_.publish(marker);
}



