#include <squirrel_object_manipulation/smash.hpp>



using namespace std;

SmashAction::SmashAction(const std::string std_SmashServerActionName) :
    smashServer(nh, std_SmashServerActionName, boost::bind(&SmashAction::executeSmash, this, _1), false),
    private_nh("~")
{
    smashServer.start();
    private_nh.param<std::string>("pose_topic", pose_topic_,"/squirrel_localizer_pose");

    pose_sub_ = nh.subscribe(pose_topic_, 2, &SmashAction::updatePose, this);
    robotino = new RobotinoControl(nh);
}

SmashAction::~SmashAction() {
    delete robotino;
}

void SmashAction::executeSmash(const squirrel_manipulation_msgs::SmashGoalConstPtr &goal) {

    
    ROS_INFO("Smash:action started \n");


    ros::Rate lRate(10);

    double d=0.23;
    double goalx, goaly;
    double Rx, Ry, Rth;
    double Vx, Vy, Vth;
    double aR2G, dR2G;

    ros::spinOnce();

    cout << "received object target position: " << goal->pose.position.x << " "  << goal->pose.position.y << "\n";
    goalx = goal->pose.position.x;
    goaly = goal->pose.position.y;

    bool smash_success_ = false;

    //main control loop
    try{

        dR2G= sqrt((Rx-goalx)*(Rx-goalx)+(Ry-goaly)*(Ry-goaly)); //d(robot, robot)
        aR2G = atan2(goaly-Ry,goalx-Rx); // angle(robot, goal)
        if (aR2G>3.14) aR2G = aR2G - 3.14;
        if (aR2G<-3.14) aR2G = aR2G + 3.14;
        if (isnan(aR2G)) aR2G = 0;

        Vth = 0; Vx = 0; Vy = 0;

        while(smash_success_){

            if (abs(aR2G-Rth)>0.1){
                Vth = 0.3 * (aR2G-Rth);
                Vx = 0; Vy = 0;
            }
            else if (dR2G<0.05){
                Vth = 0;
                Vx = 0.3 * (1.0-dR2G) ;
                Vy = 0;

            }
            else {
                smash_success_ = true;
                Vth = 0;
                Vx = 0;
                Vy = 0;
            }
        }
        robotino->singleMove(Vx,Vy,0.0,0.0,0.0,Vth);

        lRate.sleep();
        ros::spinOnce();

    }
    catch(...){
        smashResult.result_status = "failure";
        smashServer.setAborted(smashResult);
        
    }
    ros::spinOnce();

}


int main(int argc, char** argv) {

    ros::init(argc, argv, "manipulation");

    SmashAction smash(SMASH_NAME);
    ros::spin();

    return 0;

}

void SmashAction::updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg )
{
    pose_m_.x = pose_msg->pose.pose.position.x;
    pose_m_.y = pose_msg->pose.pose.position.y;
    pose_m_.theta = tf::getYaw(pose_msg->pose.pose.orientation);
}



