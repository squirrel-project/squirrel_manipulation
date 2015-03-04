#include <math.h>
#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <tf/tfMessage.h>
#include<tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <squirrel_object_manipulation/pushing.hpp>
#include <boost/assert.hpp>

using namespace std;

bool firstSet = false;
double Rx0, Ry0, Rz0;
double Ox0, Oy0, Oz0;

void arCallback(tf::tfMessage msg);

template <typename T> int sgn(T val);
double string_to_double(const std::string& s);

geometry_msgs::TransformStamped t;

PushAction::PushAction(const std::string std_PushServerActionName) :
    pushServer(nh, std_PushServerActionName, boost::bind(&PushAction::executePush, this, _1), false),
    private_nh("~")
{
    pushServer.start();
    private_nh.param<std::string>("pose_topic", pose_topic_,"/squirrel_localizer_pose");
    pose_sub_ = nh.subscribe(pose_topic_, 2, &PushAction::updatePose, this);
}

PushAction::~PushAction() {
}

void PushAction::executePush(const squirrel_manipulation_msgs::PushGoalConstPtr &goal) {



    // publish info to the console for the user
    ROS_INFO("(push up action) started push up of %s for manipulation",  goal->object_id.c_str());

    RobotinoControl robotino(nh);

    ros::Subscriber markerSub = nh.subscribe("arMarker/tf", 1, arCallback);
    ros::Rate lRate(5);

   // cout << "everything initialized" << endl;

    while(!firstSet) {
        ros::spinOnce();
        lRate.sleep();
    }

    ros::spinOnce();

    nav_msgs::Odometry Odometry = robotino.getOdom();
    double initX = Odometry.pose.pose.position.x ; double initY = Odometry.pose.pose.position.y ;

    geometry_msgs::Quaternion rotation = t.transform.rotation;

    squirrel_rgbd_mapping_msgs::GetPushingPlan srvPlan;

    ///only pusghing and object tracking -> for testing


    double x=initX;
    double y=initY;
    double th=tf::getYaw(Odometry.pose.pose.orientation);
    double yaw=th;

    ROS_INFO("current pose in odometry",  goal->object_id.c_str());

   // cout<<"start pos x:"<<x<<" y: "<<y<<" theta: "<<th<<endl<<endl<<endl;

    ROS_INFO("requesting plan");
    srvPlan.request.start.x = pose_m_.x;
    srvPlan.request.start.y = pose_m_.y;
    srvPlan.request.start.theta = pose_m_.theta;

    srvPlan.request.goal.x = goal->pose.position.x;
    srvPlan.request.goal.y = goal->pose.position.y;
    srvPlan.request.goal.theta = tf::getYaw(goal->pose.orientation);

    geometry_msgs::Point32 p1, p2, p3, p4;

    p1.x = 0.23; p1.y = -0.08;
    p2.x = 0.23; p1.y = 0.08;
    p3.x = 0.33; p1.y = -0.08;
    p4.x = 0.33; p1.y = 0.08;

    srvPlan.request.object.points.push_back(p1);
    srvPlan.request.object.points.push_back(p2);
    srvPlan.request.object.points.push_back(p3);
    srvPlan.request.object.points.push_back(p4);

    if ( ros::service::call("/getPushingPlan", srvPlan) ) {
      if ( srvPlan.response.plan.poses.empty() ) {
        ROS_WARN("got an empty plan");
      } else {
        BOOST_ASSERT_MSG( srvPlan.response.plan.header.frame_id == "/odom" ||
                          srvPlan.response.plan.header.frame_id == "odom" ,
                          "returned path is not in '/odom' frame");
        ROS_INFO("got a path for pushing");
      }
    } else {
      ROS_ERROR("unable to communicate with /getPushingPlan");
    }

    ROS_INFO("got a plan");
    nav_msgs::Path pushing_path=srvPlan.response.plan;

    int path_length= pushing_path.poses.size();

    double errX,errY,errTh;
    vector<double> X,Y,TH;

    double Xo1,Yo1,Tho1, Tho;

    int i=0;

   //getting coordinates separately

    while(i<path_length) {

        X.push_back(pushing_path.poses[i].pose.position.x);
        Y.push_back(pushing_path.poses[i].pose.position.y);
        TH.push_back(tf::getYaw(pushing_path.poses[i].pose.orientation));
        i++;
    }


    // sleep(2);

    rotation = t.transform.rotation;
    Xo1 =t.transform.translation.x; Yo1=t.transform.translation.y; Tho1=tf::getYaw(rotation);

    //path tracking
    vector<double> Ex,Ey,Eth, dEx,dEy,dEth;

    double vx,vy,omega,derrX,derrY,derrTh;
    int p=X.size();

    i=0;
    while(i<p){

        ros::spinOnce();

        Odometry = robotino.getOdom();
        x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;


        if (i==p-1){
                        ROS_INFO("recalculating plan");
                        int pom=p-i;
                        double relx, rely;
                        relx=100; rely=100;
                        for(int j=0; j<p-1; j++)
                        {
                           // cout<<"tren "<<sqrt(relx*relx+rely*rely)<<endl;
                            //cout<<"moguca "<<sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy))<<endl;
                                if (sqrt(relx*relx+rely*rely)>sqrt((X[j]-x)*(X[j]-x)+(Y[j]-y)*(Y[j]-y)))
                                {
                                    relx=X[j]-x;
                                    rely=Y[j]-y;
                                    pom=j;
                                }
                        }
                        i=pom;
                    }

       errX=X[i]-x;
       errY=Y[i]-y;
       yaw=tf::getYaw(Odometry.pose.pose.orientation);
       errTh=TH[i]-yaw;
       th=yaw;

       rotation = t.transform.rotation;


       Ex.push_back(errX);
       Ey.push_back(errY);
       Eth.push_back(errTh);
       if(i==1){
           derrX=0; derrY=0; derrTh=0;
       }
       else{
           derrX=errX-Ex[i-1];//derivatives
           derrY=errY-Ey[i-1];
           derrTh=errTh-Eth[i-1];
       }


       dEx.push_back(derrX);
       dEy.push_back(derrY);
       dEth.push_back(derrTh);

       vx=(0.4*(errX*cos(th)+errY*sin(th))+0.1*(derrX*cos(th)+derrY*sin(th)));
       vy=0.2*(errY*cos(th)-errX*sin(th))+0.1*(derrY*cos(th)-derrX*sin(th));
       omega=0.5*errTh;

       Tho=tf::getYaw(rotation);

       if(sgn(t.transform.translation.x)*t.transform.translation.x>0.02){
           robotino.singleMove( 0, -1*t.transform.translation.x, 0, 0, 0, omega);
           i=i-1;

       }
      // else*/
       /*    if (sgn(errTh)*errTh> 0.05){
           robotino.singleMove( 0, 0, 0, 0, 0, omega);


       }*/
      else if (sgn(errY)*errY> 0.05){
           robotino.singleMove( vx, vy, 0, 0, 0, 0);

       }
       else
       {
        robotino.singleMove( vx, 0, 0, 0, 0, 0);

       }

       i++;
       lRate.sleep();

     }


    /* for(int i = 1; i <= 10; ++i) {

        rate.sleep();
        PushFeedback.percent_completed = i * 10;
        pushServer.publishFeedback(PushFeedback);

    }*/

    pushResult.result_status = "done";
    pushServer.setSucceeded(pushResult);

}


void PushAction::updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg )
{
  pose_m_.x = pose_msg->pose.pose.position.x;
  pose_m_.y = pose_msg->pose.pose.position.y;
  pose_m_.theta = tf::getYaw(pose_msg->pose.pose.orientation);
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "manipulation");

    PushAction push(PUSH_NAME);
    ros::spin();

    return 0;

}



void arCallback(tf::tfMessage msg) {

    if (!firstSet){
    firstSet = true;
    }

     t = msg.transforms.at(0);

}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double string_to_double(const std::string& s) {
    std::istringstream i(s);
    double x;
    if (!(i >> x))
        return 0;
    return x;
}

