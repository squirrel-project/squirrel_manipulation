#include <math.h>
#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include "squirrel_object_manipulation/RobotinoControl.hpp"
#include "squirrel_rgbd_mapping_msgs/GetPushingPlan.h"

using namespace std;
using namespace ros;



bool firstSet = false;
double Rx0, Ry0, Rz0;
double Ox0, Oy0, Oz0;

void arCallback(tf::tfMessage msg);

template <typename T> int sgn(T val);
double string_to_double(const std::string& s);

geometry_msgs::TransformStamped t;

int main(int argc, char** args) {

   

    ros::init(argc, args, "push");
    ros::NodeHandle node;
    usleep(1e6);

  
    RobotinoControl robotino(node);

    ros::Subscriber markerSub = node.subscribe("arMarker/tf", 1, arCallback);
    ros::Rate lRate(5);

    cout << "everything initialized" << endl;

    while(!firstSet) {
        ros::spinOnce();
        lRate.sleep();
    }

    ros::spinOnce();
  
    nav_msgs::Odometry Odometry = robotino.getOdom();
    double initX = Odometry.pose.pose.position.x ; double initY = Odometry.pose.pose.position.y ; double initZ = Odometry.pose.pose.position.z ;
   
    geometry_msgs::Quaternion rotation = t.transform.rotation;
    
    squirrel_rgbd_mapping_msgs::GetPushingPlan srvPlan;
    
    
    

    ///only pusghing and object tracking -> for tetsing


    double x=initX; double  y=initY;  double th=tf::getYaw(Odometry.pose.pose.orientation);
    double  initth=th;
    double  yaw=th;
    

    cout<<"start pos x:"<<x<<" y: "<<y<<" theta: "<<th<<endl<<endl<<endl;
    
    srvPlan.request.start.x=initX;
    srvPlan.request.start.y=initY;
    srvPlan.request.start.theta=th;
    

     double errX,errY,errTh, eOX,eOY,eOTh;
     vector<double> X,Y,TH,Xo,Yo,THo;

     double Xo1,Yo1,Tho1, Tho;

     //pushing on the path

    x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;


    sleep(2);

 
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // call service for geting vector of X, Y and TH
    //apply transformation matrix to set it in odometry??

    double n = 0;

    /*while(!inFile.eof()) {

        for(int i = 0; i < 3; ++i) {
           inFile>>val;

         //  cout<<val<<endl;

            n = string_to_double(val);
            if(i == 0) { X.push_back(n); }
            else if(i == 1) { Y.push_back(n); }
            else { TH.push_back(n); }
        }
        fflush(stdout);
    }*/



    rotation = t.transform.rotation;
    Xo1 =t.transform.translation.x; Yo1=t.transform.translation.y; Tho1=tf::getYaw(rotation);

    //path tracking
    vector<double> Ex,Ey,Eth, dEx,dEy,dEth, Eox,Eoy,EoTh;

    double vx,vy,omega,derrX,derrY,derrTh;
    int p=X.size();
    int i=0;
    while(i<p){

        ros::spinOnce();

        Odometry = robotino.getOdom();
        x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;



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

       vx=(0.4*(errX*cos(th)+errY*sin(th))+0.1*(derrX*cos(th)+derrY*sin(th)));//*sgn(errX*cos(th)+errY*sin(th))+0.1*(derrX*cos(th)+derrY*sin(th));
       vy=0.2*(errY*cos(th)-errX*sin(th))+0.1*(derrY*cos(th)-derrX*sin(th));
       omega=0.5*errTh;

       Tho=tf::getYaw(rotation);

      /* if(sgn(t.transform.translation.x)*t.transform.translation.x>0.02){
           robotino.singleMove( 0, -1*t.transform.translation.x, 0, 0, 0, omega);
           i=i-1;

       }*/
       /* else if(sgn(Tho)*Tho>0.3){
           robotino.singleMove(0, -1*Tho, 0, 0, 0, omega);
           i=i-1;

       }*/
       if(sgn(t.transform.translation.x)*t.transform.translation.x>0.02){
           robotino.singleMove( 0, -1*t.transform.translation.x, 0, 0, 0, omega);
           i=i-1;

       }
       else if (sgn(errTh)*errTh> 0.05){
           robotino.singleMove( 0, 0, 0, 0, 0, omega);
          
        
       }
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
