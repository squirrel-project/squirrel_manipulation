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
#include <robotino_msgs/ResetOdometry.h>
#include "squirrel_object_manipulation/RobotinoControl.hpp"

using namespace std;
using namespace ros;


string expN="trajOlog3";

//string tr="trajObjectL.txt";
//string tr="trajL2.txt"; //for robot
//string tr="trajL4.txt"; //for object
string tr="trajO.txt";
bool firstSet = false;
bool object_lost=false;
double Rx0, Ry0, Rz0;
double Ox0, Oy0, Oz0;
double offsetTh, offsetX,offsetY; // ar tags calibration. inital offset
double offsetTh1, offsetX1,offsetY1;
double temp=0;
double deltat=1e-1;


void arCallback(tf::tfMessage msg);

void update_robot_coordiantes(RobotinoControl *robotino);
bool update_object_coordiantes();
void face_object(RobotinoControl *robotino, double aR2O);

template <typename T> int sgn(T val);
double string_to_double(const std::string& s);

vector<double> Ex,Ey,Eth, dEx,dEy,dEth, Eox,Eoy,EoTh;

double vx,vy,omega,derrX,derrY,derrTh, Ox,Oy,Oth,Olx,Oly,Olth,Ox1,Oy1,Oth1, x,y,th;

geometry_msgs::TransformStamped t;

int main(int argc, char** args) {

    int execute;
   // execute=0; //collect trajectory in odometry
   // execute=1; //modified pid with fixed parameters. path tracking for robot in odometry
    //execute=2; //modified pid with fixed parameters. path tracking for objectu in odometry
   //execute=3; //testing object tracking
   // execute=4; //elipse movement
   // execute=5;// modified pid with fixed parameters. path tracking for robot in odometry + elipse movement //hardcoded some parameters
    //execute=6;// modified pid with fixed parameters. path tracking for object in odometry + elipse movement //hardcoded some parameters
  //execute=7;// modified pid with fixed parameters. path tracking for object in odometry + elipse movement //hardcoded some parameters
              //added reaching the goal
              //added detection of loosing object
    // execute=8; //test ar tags
    // execute =9; //test circle and direction placement
   // execute =10;

    // execute=11; //new circle movement
    // execute=12;
    //execute=13;
    execute=14;



    ros::init(argc, args, "push");
    ros::NodeHandle node;
    usleep(1e6);



    ros::ServiceClient rO=node.serviceClient<robotino_msgs::ResetOdometry>("/reset_odometry");
    robotino_msgs::ResetOdometry R;
    R.request.x=0;
    R.request.y=0;
    R.request.phi=0;

    if( rO.call(R))
    {cout<<"call success"<<endl;}
    else{ cout<<"call not successful"<<endl; }




   RobotinoControl robotino(node);



    ros::Subscriber markerSub = node.subscribe("arMarker/tf", 1, arCallback);
    //ros::Subscriber markerSub = node.subscribe("tf", 1, arCallback);
    ros::Rate lRate(5);

    cout << "everything initialized" << endl;

    while(!firstSet) {
        ros::spinOnce();
        lRate.sleep();
    }

    double d=0.21; //robotino diameter

   offsetX=t.transform.translation.x;
    offsetY=t.transform.translation.y;
    offsetTh=tf::getYaw(t.transform.rotation);

    offsetX1=t.transform.translation.x;
    offsetY1=t.transform.translation.y+d;
    offsetTh1=tf::getYaw(t.transform.rotation);

    cout<<"Offest for x is: "<<offsetX<<endl;
    cout<<"Offset for y is: "<<offsetY<<endl;
    cout<<"Offset for th is: "<<offsetTh<<endl<<endl;

    cout<<"Offest for x1 is: "<<offsetX1<<endl;
    cout<<"Offset for y1 is: "<<offsetY1<<endl;
    cout<<"Offset for th1 is: "<<offsetTh1<<endl<<endl;


    ros::spinOnce();
    cout << "here" << endl;
    nav_msgs::Odometry Odometry = robotino.getOdom();
    double initX = Odometry.pose.pose.position.x ; double initY = Odometry.pose.pose.position.y ; double initZ = Odometry.pose.pose.position.z ;
    //double initOx=Odometry.pose.pose.orientation.x ;  double initOy=Odometry.pose.pose.orientation.y;  double initOz=Odometry.pose.pose.orientation.z;

    geometry_msgs::Quaternion rotation = t.transform.rotation;

    ///only pushing and object tracking -> for tetsing

   /*while(1){
        ros::spinOnce();

    rotation = t.transform.rotation;
    cout<<"testing : TAG: x: " <<t.transform.translation.x<<" y: "<<t.transform.translation.y<<" z: "<<t.transform.translation.z<<" yaw: "<<tf::getYaw(rotation)<<endl<<endl;

    robotino.singleMove( -0.5*t.transform.translation.y, -0.5*t.transform.translation.x, 0, 0, 0, 0);
    robotino.singleMove( t.transform.translation.y, -0.5*t.transform.translation.x, 0, 0, 0, 0);
    lRate.sleep();
    }*/



    update_robot_coordiantes(&robotino);
    //double x=initX; double  y=initY;  double th=tf::getYaw(Odometry.pose.pose.orientation);
    double  initth=th;
    double  yaw=th;

    cout<<"start pos x:"<<x<<" y: "<<y<<" z: "<<initZ<<" theta: "<<th<<endl<<endl<<endl;

     double errX,errY,errTh, eOX,eOY,eOTh;
     vector<double> X,Y,TH,Xo,Yo,THo;

     double Xo1,Yo1,Tho1, Tho;

     switch (execute){
     case 0:
     {
         string traj="/home/c7031098/testing/robotino/"+tr;
         ofstream oFile;
         oFile.open(traj.c_str());
         cout<<"file open"<<endl;

         //performing desired path
        while(x<initX+0.4){

            ros::spinOnce();

            Odometry = robotino.getOdom();
            x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;
            X.push_back(x);
            Y.push_back(y);

            yaw=tf::getYaw(Odometry.pose.pose.orientation);
            TH.push_back(yaw);

          //   rotation = t.transform.rotation;
          //   cout<<"TAG: x: " <<t.transform.translation.x<<" y: "<<t.transform.translation.y<<" z: "<<t.transform.translation.z<<" yaw: "<<tf::getYaw(rotation)<<endl<<endl;

             oFile << "\t" << x << "\t" << y << "\t" <<yaw<<endl;

             robotino.singleMove( 0.05, 0, 0, 0, 0, 0);
           // cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<" yaw "<<yaw<< endl<<endl<<endl;
            lRate.sleep();

       }

        cout<<"phase 1 done"<<endl;
        while(yaw>initth-1.50){

            ros::spinOnce();

            Odometry = robotino.getOdom();
            x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;
            X.push_back(x);
            Y.push_back(y);
            yaw=tf::getYaw(Odometry.pose.pose.orientation);
             TH.push_back(yaw);

             robotino.singleMove( 0, 0, 0, 0, 0, -0.05);

              oFile << "\t" << x << "\t" << y << "\t" <<yaw<<endl;

           // cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<" yaw "<<yaw<< endl<<endl<<endl;

            //cout<<"current pos x:"<<x<<" y: "<<y<<" z: "<<z<<endl<<endl<<endl;
            lRate.sleep();

       }

         cout<<"phase 2 done"<<endl;

       // cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<endl<<endl<<endl<<endl<<endl;

        while(y>-0.4){

            ros::spinOnce();

            Odometry = robotino.getOdom();
            x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;

            X.push_back(x);
            Y.push_back(y);
            yaw=tf::getYaw(Odometry.pose.pose.orientation);
             TH.push_back(yaw);

             robotino.singleMove( 0.05, 0, 0, 0, 0, 0);

             oFile << "\t" << x << "\t" << y << "\t" <<yaw<<endl;

           // cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<" yaw "<<yaw<< endl<<endl<<endl;

            //cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<endl;
            lRate.sleep();
             cout<<"phase 3 done"<<endl;

       }

         break;
     }



     case 1:
     {

    x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;


    sleep(2);

    if( rO.call(R))
    {cout<<"call success"<<endl;}
    else{ cout<<"call not successful"<<endl; }
    sleep(5);

    cout<<"PUT THE OBJECT IN FRONT OF THE ROBOT"<<endl<<endl;


    //reading from file
    ifstream inFile;
    string filei="/home/c7031098/testing/robotino/"+tr;
    inFile.open (filei.c_str(),ios::in);
    string val;

    string pathR="/home/c7031098/testing/robotino/pathR"+expN;
    ofstream oFileR;
    oFileR.open(pathR.c_str());

    string errR="/home/c7031098/testing/robotino/errR"+expN;
    ofstream oFileER;
    oFileER.open(errR.c_str());

    string velR="/home/c7031098/testing/robotino/velR"+expN;
    ofstream oFileVR;
    oFileVR.open(velR.c_str());

    string pathO="/home/c7031098/testing/robotino/pathO"+expN;
    ofstream oFileO;
    oFileO.open(pathO.c_str());

    cout<<"file open"<<endl;

    double n = 0;

    while(!inFile.eof()) {

        for(int i = 0; i < 3; ++i) {
           inFile>>val;

         //  cout<<val<<endl;

            n = string_to_double(val);
            if(i == 0) { X.push_back(n); }
            else if(i == 1) { Y.push_back(n); }
            else { TH.push_back(n); }
        }
        fflush(stdout);
    }



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
      //S cout<<"TAG: x: " <<t.transform.translation.x<<" y: "<<t.transform.translation.y<<" z: "<<t.transform.translation.z<<" yaw: "<<tf::getYaw(rotation)<<endl<<endl;


      // cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<" yaw "<<yaw<< endl<<endl<<endl;


      // cout<<"greske : x: "<<errX<<"y:  "<<errY<<" th "<<errTh<<endl<<endl;


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

       oFileR << "\t" << x << "\t" << y << "\t" <<yaw<<endl;
       oFileO << "\t" << t.transform.translation.x << "\t" << t.transform.translation.y << "\t" <<Tho<<endl;
       oFileER << "\t" << errX << "\t" << errY << "\t" <<errTh<<endl;

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
            oFileVR << "\t" << 0 << "\t" << -1*t.transform.translation.x << "\t" <<omega<<endl;

       }
       else if (sgn(errTh)*errTh> 0.5){
           robotino.singleMove( 0, 0, 0, 0, 0, omega);
            oFileVR << "\t" << 0 << "\t" << 0 << "\t" <<omega<<endl;
         //  cout<<"here 1"<<endl;
       }
       else if (sgn(errY)*errY> 0.05){
           robotino.singleMove( vx, vy, 0, 0, 0, 0);
            oFileVR << "\t" << vx << "\t" << vy << "\t" <<0<<endl;
          // cout<<"here 2"<<endl;
       }
       else
       {
        robotino.singleMove( vx, 0, 0, 0, 0, 0);
        oFileVR << "\t" <<vx << "\t" << 0 << "\t" <<0<<endl;
       }


       i++;
       lRate.sleep();


     }
    break;
     }


         //control only by obesrving position of the opecjt and keeping object on the path
         //fixed controller parameters

     case 2:
     {

    x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;

    sleep(2);

    //calling reset of odometry

    if( rO.call(R))
    {cout<<"odometry reset: call success"<<endl;}
    else{ cout<<"call not successful"<<endl; }
    sleep(5);

    cout<<"PUT THE OBJECT IN FRONT OF THE ROBOT FOR CALIBRATION"<<endl<<endl;


    //reading from file
    ifstream inFile;
    string filei="/home/c7031098/testing/robotino/"+tr;
    inFile.open (filei.c_str(),ios::in);
    string val;

    //writing results to files

    string pathR="/home/c7031098/testing/robotino/pathR"+expN;
    ofstream oFileR;
    oFileR.open(pathR.c_str());

    string errR="/home/c7031098/testing/robotino/errR"+expN;
    ofstream oFileER;
    oFileER.open(errR.c_str());

    string velR="/home/c7031098/testing/robotino/velR"+expN;
    ofstream oFileVR;
    oFileVR.open(velR.c_str());

    string pathO="/home/c7031098/testing/robotino/pathO"+expN;
    ofstream oFileO;
    oFileO.open(pathO.c_str());

    cout<<"file open"<<endl;

    double n = 0;

    while(!inFile.eof()) {

        for(int i = 0; i < 3; ++i) {
           inFile>>val;

         //  cout<<val<<endl;

            n = string_to_double(val);
            if(i == 0) { X.push_back(n); }
            else if(i == 1) { Y.push_back(n); }
            else { TH.push_back(n); }
        }
        fflush(stdout);
    }



   // rotation = t.transform.rotation;
    //Xo1 =t.transform.translation.x; Yo1=t.transform.translation.y; Tho1=tf::getYaw(rotation);

    //path tracking
    vector<double> Ex,Ey,Eth, dEx,dEy,dEth, Eox,Eoy,EoTh;

    double vx,vy,omega,derrX,derrY,derrTh, Ox,Oy,Oth,Olx,Oly,Olth;
    int p=X.size();
    int i=0;
    while(i<p){

        ros::spinOnce();

        //getting position o fthe robot in /odom

        Odometry = robotino.getOdom();
        x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;
        th=tf::getYaw(Odometry.pose.pose.orientation);

        cout<<"robot position: "<<"x: "<<x<<"y: "<<y<<"th "<<th<<endl;



        //t contains relative position of object to the robot

        //shifting object position to calibrated artag system
        Olx=t.transform.translation.x -offsetX;
        Oly=t.transform.translation.y -offsetY;
        Olth=tf::getYaw(t.transform.rotation) -offsetTh;

        cout<<"relative object position: "<<"x: "<<Olx<<"y: "<<Oly<<"th "<<Oth<<endl;

         //calculating global position of object

       /* Ox=x - Olx*sin(th)-Oly*cos(th); //ver 1
        Oy=y - Olx*cos(th)+Oly*sin(th);
        Oth=th - Olth;*/

        Ox=x + Olx*sin(th)-Oly*cos(th); //ver 2
        Oy=y - Olx*cos(th)-Oly*sin(th);
        Oth=th - Olth;
        cout<<"global object position: "<<"x: "<<Ox<<"y: "<<Oy<<"th "<<Oth<<endl<<endl;

       // Ox=x- ( t.transform.translation.y*cos(th)+(t.transform.;translation.x-offsetX)*sin(th));
        //Oy=y- ((t.transform.translation.x-offsetX)*cos(th)-t.transform.translation.y*sin(th));
        //rotation = t.transform.rotation;

       // Ox=x- ( t.transform.translation.y*cos(th)-(t.transform.translation.x-offsetX)*sin(th));
        //Oy=y- ((t.transform.translation.x-offsetX)*cos(th)+t.transform.translation.y*sin(th));
        //Oth = th-tf::getYaw(t.transform.rotation);
     //S   Oth=th +tf::getYaw(rotation);

        // calculating errors

       errX=X[i]-Ox;
       errY=Y[i]-Oy;
       errTh=TH[i]-Oth;


       Ex.push_back(errX);
       Ey.push_back(errY);
       Eth.push_back(errTh);

       if(i==1){
           derrX=0; derrY=0; derrTh=0;
       }
       else{
           derrX=errX-Ex[i-1];// calculating differences for derivatives
           derrY=errY-Ey[i-1];
           derrTh=errTh-Eth[i-1];
       }

       dEx.push_back(derrX);
       dEy.push_back(derrY);
       dEth.push_back(derrTh);


       vx=(0.4*(errX*cos(th)+errY*sin(th))+0.1*(derrX*cos(th)+derrY*sin(th)));
       vy=0.2*(errY*cos(th)-errX*sin(th))+0.1*(derrY*cos(th)-derrX*sin(th));
       omega=0.5*errTh;


      /* if(sgn(t.transform.translation.x)*t.transform.translation.x>0.02){
           robotino.singleMove( 0, -1*t.transform.translation.x, 0, 0, 0, omega);
           i=i-1;

       }*/
       /* else if(sgn(Tho)*Tho>0.3){
           robotino.singleMove(0, -1*Tho, 0, 0, 0, omega);
           i=i-1;

       }*/
      /* if(sgn(t.transform.translation.x)*t.transform.translation.x>0.02){
           robotino.singleMove( 0, -1*t.transform.translation.x, 0, 0, 0, omega);
           i=i-1;
            oFileVR << "\t" << 0 << "\t" << -1*t.transform.translation.x << "\t" <<omega<<endl;

       }*/
       if (sgn(errY)*errY> 0.01){
           robotino.singleMove( 0, -vy, 0, 0, 0, 0);
            oFileVR << "\t" << vx << "\t" << vy << "\t" <<0<<endl;
          // cout<<"here 2"<<endl;
       }

       else if (sgn(errY)*errY> 0.01){
           robotino.singleMove( vx, vy, 0, 0, 0, 0);
            oFileVR << "\t" << vx << "\t" << vy << "\t" <<0<<endl;
          // cout<<"here 2"<<endl;
       }
       else if (sgn(errTh)*errTh> 0.05){
           robotino.singleMove( 0, 0, 0, 0, 0, omega);
            oFileVR << "\t" << 0 << "\t" << 0 << "\t" <<omega<<endl;
         //  cout<<"here 1"<<endl;
       }
       else
       {
        robotino.singleMove( vx, 0, 0, 0, 0, 0);
        oFileVR << "\t" <<vx << "\t" << 0 << "\t" <<0<<endl;
       }


       i++;
       lRate.sleep();


     }
    break;
     }
     case 3:
     {

    x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;

    sleep(2);


    if( rO.call(R))
    {cout<<"odometry reset: call success"<<endl;}
    else{ cout<<"call not successful"<<endl; }
    sleep(5);

    cout<<"PUT THE OBJECT IN FRONT OF THE ROBOT FOR CALIBRATION"<<endl<<endl;


    int p=X.size();
    int i=0;
    while(1){

        ros::spinOnce();

        //getting position o fthe robot in /odom

        Odometry = robotino.getOdom();
        x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;
        th=tf::getYaw(Odometry.pose.pose.orientation);

        cout<<"robot position: "<<" x: "<<x<<" y: "<<y<<" th "<<th<<endl;


        Olx=t.transform.translation.x -offsetX;
        Oly=t.transform.translation.y -offsetY;
        Olth=tf::getYaw(t.transform.rotation) -offsetTh;

       /* Olx=t.transform.translation.x;
        Oly=t.transform.translation.y ;
        Olth=tf::getYaw(t.transform.rotation) ;*/


        cout<<"relative object position: "<<" x: "<<Olx<<" y: "<<Oly<<" th "<<Oth<<endl;

         //calculating global position of object

        Ox1=x - Olx*sin(th)+Oly*cos(th); //ver 1
        Oy1=y + Olx*cos(th)+Oly*sin(th);
        Oth1=th - Olth;

        Ox=x - Olx*sin(th)-Oly*cos(th); //ver 2
        Oy=y - Olx*cos(th)+Oly*sin(th);
        Oth=th - Olth;
      double Ox2,Oy2,Oth2;

        Ox2=x - Oly*cos(th)-Olx*sin(th); //ver 2
        Oy2=y + Oly*sin(th)-Olx*cos(th);
        Oth2=th - Olth;

        cout<<"global object position: "<<" x: "<<Ox<<" y: "<<Oy<<" th "<<Oth<<endl<<endl;
        cout<<"global object position1: "<<" x: "<<Ox1<<" y: "<<Oy1<<" th "<<Oth1<<endl<<endl;
        cout<<"global object position: "<<" x: "<<Ox2<<" y: "<<Oy2<<" th "<<Oth2<<endl<<endl;


       // Ox=x- ( t.transform.translation.y*cos(th)+(t.transform.;translation.x-offsetX)*sin(th));
        //Oy=y- ((t.transform.translation.x-offsetX)*cos(th)-t.transform.translation.y*sin(th));
        //rotation = t.transform.rotation;

       // Ox=x- ( t.transform.translation.y*cos(th)-(t.transform.translation.x-offsetX)*sin(th));
        //Oy=y- ((t.transform.translation.x-offsetX)*cos(th)+t.transform.translation.y*sin(th));
        //Oth = th-tf::getYaw(t.transform.rotation);
     //S   Oth=th +tf::getYaw(rotation);



       lRate.sleep();



     }
    break;
     }
     case 4: {

     double e1=0.1;
     double e2=0.1;
     double t=0;

     double deltat=1e-1;
     cout<< deltat<<endl;

    //cout<< "here 1"<<endl;

     while(t<3.2){
         //  cout<< "here 2"<<endl;
         ros::spinOnce();
         //  cout<< "here 3"<<endl;
       // robotino.singleMove(-1* e1*cos(t), -1*e2*sin(t), 0, 0, 0, 0); //udesno i unazad
         robotino.singleMove( -1*e1*cos(t), e2*sin(t), 0, 0, 0, -0.05);

         cout <<"cos "<<cos(t)<<" sin "<<sin(t)<<endl;
         t=t+deltat;
         lRate.sleep();

     }


     break;
     }

     case 5:
     {

    x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;


    sleep(2);

    if( rO.call(R))
    {cout<<"call success"<<endl;}
    else{ cout<<"call not successful"<<endl; }
    sleep(5);

    cout<<"PUT THE OBJECT IN FRONT OF THE ROBOT"<<endl<<endl;


    //reading from file
    ifstream inFile;
    string filei="/home/c7031098/testing/robotino/"+tr;
    inFile.open (filei.c_str(),ios::in);
    string val;

    string pathR="/home/c7031098/testing/robotino/pathR"+expN;
    ofstream oFileR;
    oFileR.open(pathR.c_str());

    string errR="/home/c7031098/testing/robotino/errR"+expN;
    ofstream oFileER;
    oFileER.open(errR.c_str());

    string velR="/home/c7031098/testing/robotino/velR"+expN;
    ofstream oFileVR;
    oFileVR.open(velR.c_str());

    string pathO="/home/c7031098/testing/robotino/pathO"+expN;
    ofstream oFileO;
    oFileO.open(pathO.c_str());

    cout<<"file open"<<endl;

    double n = 0;

    while(!inFile.eof()) {

        for(int i = 0; i < 3; ++i) {
           inFile>>val;

         //  cout<<val<<endl;

            n = string_to_double(val);
            if(i == 0) { X.push_back(n); }
            else if(i == 1) { Y.push_back(n); }
            else { TH.push_back(n); }
        }
        fflush(stdout);
    }



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

       Olx=t.transform.translation.x -offsetX;
        Oly=t.transform.translation.y -offsetY;
       Olth=tf::getYaw(t.transform.rotation) -offsetTh;
      //S cout<<"TAG: x: " <<t.transform.translation.x<<" y: "<<t.transform.translation.y<<" z: "<<t.transform.translation.z<<" yaw: "<<tf::getYaw(rotation)<<endl<<endl;


      // cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<" yaw "<<yaw<< endl<<endl<<endl;


      // cout<<"greske : x: "<<errX<<"y:  "<<errY<<" th "<<errTh<<endl<<endl;


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

       oFileR << "\t" << x << "\t" << y << "\t" <<yaw<<endl;
       oFileO << "\t" <<Olx << "\t" <<Oly << "\t" <<Olth<<endl;
       oFileER << "\t" << errX << "\t" << errY << "\t" <<errTh<<endl;


      if(sgn(Olx)*Olx>0.08){ //odl value 0.08
          temp=0;
          while(temp<3.2){
              //  cout<< "here 2"<<endl;
              ros::spinOnce();
              //  cout<< "here 3"<<endl;
              if(sgn(Olx)>0) robotino.singleMove(-1* 0.1*Oly*cos(temp), -1*0.1*Olx*sin(temp), 0, 0, 0, 0.05); //udesno i unazad
            else robotino.singleMove( -1*0.1*Oly*cos(temp), 0.1*Olx*sin(temp), 0, 0, 0, -0.05);

             // cout <<"cos "<<cos(temp)<<" sin "<<sin(temp)<<endl;
              temp=temp+deltat;
              lRate.sleep();

          }

      }
      else if(sgn(Olx)*Olx>0.02){
           robotino.singleMove( 0, -1*Olx, 0, 0, 0, omega);
           i=i-1;
            oFileVR << "\t" << 0 << "\t" << -1*Olx<< "\t" <<omega<<endl;

       }
       else if (sgn(errTh)*errTh> 0.5){
           robotino.singleMove( 0, 0, 0, 0, 0, omega);
            oFileVR << "\t" << 0 << "\t" << 0 << "\t" <<omega<<endl;
         //  cout<<"here 1"<<endl;
       }
       else if (sgn(errY)*errY> 0.05){
           robotino.singleMove( vx, vy, 0, 0, 0, 0);
            oFileVR << "\t" << vx << "\t" << vy << "\t" <<0<<endl;
          // cout<<"here 2"<<endl;
       }
       else
       {
        robotino.singleMove( vx, 0, 0, 0, 0, 0);
        oFileVR << "\t" <<vx << "\t" << 0 << "\t" <<0<<endl;
       }


       i++;
       lRate.sleep();


     }
    break;
     }
     case 6:
     {

    x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;

    sleep(2);

    if( rO.call(R))
    {cout<<"call success"<<endl;}
    else{ cout<<"call not successful"<<endl; }
    sleep(5);

    cout<<"PUT THE OBJECT IN FRONT OF THE ROBOT"<<endl<<endl;

    //reading from file
    ifstream inFile;
    string filei="/home/c7031098/testing/robotino/"+tr;
    inFile.open (filei.c_str(),ios::in);
    string val;

    string pathR="/home/c7031098/testing/robotino/pathR"+expN;
    ofstream oFileR;
    oFileR.open(pathR.c_str());

    string errR="/home/c7031098/testing/robotino/errR"+expN;
    ofstream oFileER;
    oFileER.open(errR.c_str());

    string velR="/home/c7031098/testing/robotino/velR"+expN;
    ofstream oFileVR;
    oFileVR.open(velR.c_str());

    string pathO="/home/c7031098/testing/robotino/pathO"+expN;
    ofstream oFileO;
    oFileO.open(pathO.c_str());

    cout<<"file open"<<endl;

    double n = 0;

    while(!inFile.eof()) {

        for(int i = 0; i < 3; ++i) {
           inFile>>val;
            n = string_to_double(val);
            if(i == 0) { X.push_back(n); }
            else if(i == 1) { Y.push_back(n); }
            else { TH.push_back(n); }
        }
        fflush(stdout);
    }



    rotation = t.transform.rotation;
    Xo1 =t.transform.translation.x; Yo1=t.transform.translation.y; Tho1=tf::getYaw(rotation);

    //path tracking
    vector<double> Ex,Ey,Eth, dEx,dEy,dEth;

    double vx,vy,omega,derrX,derrY,derrTh;
    int p=X.size();
    int i=0;
    while(i<p){

        ros::spinOnce();
        //robotino position in /odom world
        Odometry = robotino.getOdom();
        x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;
        th=tf::getYaw(Odometry.pose.pose.orientation);

        //relative position of the object
       Olx=t.transform.translation.x -offsetX;
       Oly=t.transform.translation.y -offsetY;
       Olth=tf::getYaw(t.transform.rotation) -offsetTh;

       //global position of object in /odom
       Ox=x - Oly*cos(th)-Olx*sin(th);
       Oy=y + Oly*sin(th)-Olx*cos(th);
       Oth=th - Olth;


       //calculating errors

       errTh=TH[i]-Oth;
       errX=X[i]-Ox;
       errY=Y[i]-Oy;


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

       oFileR << "\t" << x << "\t" << y << "\t" <<yaw<<endl;
       oFileO << "\t" <<Olx << "\t" <<Oly << "\t" <<Olth<<endl;
       oFileER << "\t" << errX << "\t" << errY << "\t" <<errTh<<endl;


      if(sgn(Olx)*Olx>0.08){
          temp=0;
          while(temp<3.2){
              //  cout<< "here 2"<<endl;
              ros::spinOnce();
              //  cout<< "here 3"<<endl;
              if(sgn(Olx)>0) robotino.singleMove(-1* 0.1*Oly*cos(temp), -1*0.1*Olx*sin(temp), 0, 0, 0, 0.05); //udesno i unazad
            else robotino.singleMove( -1*0.1*cos(temp), 0.1*sin(temp), 0, 0, 0, -0.05);

              cout <<"cos "<<cos(temp)<<" sin "<<sin(temp)<<endl;
              temp=temp+deltat;
              lRate.sleep();

          }

      }
      else if(sgn(Olx)*Olx>0.02){
           robotino.singleMove( 0, -1*Olx, 0, 0, 0, omega);
           i=i-1;
            oFileVR << "\t" << 0 << "\t" << -1*Olx<< "\t" <<omega<<endl;

       }
       else if (sgn(errTh)*errTh> 0.5){
           robotino.singleMove( 0, 0, 0, 0, 0, omega);
            oFileVR << "\t" << 0 << "\t" << 0 << "\t" <<omega<<endl;
         //  cout<<"here 1"<<endl;
       }
       else if (sgn(errY)*errY> 0.05){
           robotino.singleMove( vx, vy, 0, 0, 0, 0);
            oFileVR << "\t" << vx << "\t" << vy << "\t" <<0<<endl;
          // cout<<"here 2"<<endl;
       }
       else
       {
        robotino.singleMove( vx, 0, 0, 0, 0, 0);
        oFileVR << "\t" <<vx << "\t" << 0 << "\t" <<0<<endl;
       }


       i++;
       lRate.sleep();


     }
    break;
     }
     case 7:
          {

         x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;

         sleep(2);

         if( rO.call(R))
         {cout<<"call success"<<endl;}
         else{ cout<<"call not successful"<<endl; }
         sleep(5);

         cout<<"PUT THE OBJECT IN FRONT OF THE ROBOT"<<endl<<endl;

         //reading from file
         ifstream inFile;
         string filei="/home/c7031098/testing/robotino/"+tr;
         inFile.open (filei.c_str(),ios::in);
         string val;

         string pathR="/home/c7031098/testing/robotino/pathR"+expN;
         ofstream oFileR;
         oFileR.open(pathR.c_str());

         string errR="/home/c7031098/testing/robotino/errR"+expN;
         ofstream oFileER;
         oFileER.open(errR.c_str());

         string velR="/home/c7031098/testing/robotino/velR"+expN;
         ofstream oFileVR;
         oFileVR.open(velR.c_str());

         string pathO="/home/c7031098/testing/robotino/pathO"+expN;
         ofstream oFileO;
         oFileO.open(pathO.c_str());

         cout<<"file open"<<endl;

         double n = 0;

         while(!inFile.eof()) {

             for(int i = 0; i < 3; ++i) {
                inFile>>val;
                 n = string_to_double(val);
                 if(i == 0) { X.push_back(n); }
                 else if(i == 1) { Y.push_back(n); }
                 else { TH.push_back(n); }
             }
             fflush(stdout);
         }



         rotation = t.transform.rotation;
         Xo1 =t.transform.translation.x; Yo1=t.transform.translation.y; Tho1=tf::getYaw(rotation);

         //path tracking
         vector<double> Ex,Ey,Eth, dEx,dEy,dEth,vOlx,vOly,vOlth;

         double vx,vy,omega,derrX,derrY,derrTh;
         int p=X.size();
         int i=0;
         while ((i<p-1)||((i==p-1)&&((abs(X[p-1]-Ox)>0.05)||(abs(Y[p-1]-Oy)>0.05)))){ //error tolerance of 5 cm

             ros::spinOnce();
             //robotino position in /odom world
             Odometry = robotino.getOdom();
             x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;
             th=tf::getYaw(Odometry.pose.pose.orientation);

             //relative position of the object
             int check=0;
             while(!check){
            Olx=t.transform.translation.x -offsetX;
            Oly=t.transform.translation.y -offsetY;
            Olth=tf::getYaw(t.transform.rotation) -offsetTh;
            if(!isnan(Olth))check=1;
            else {
                cout<<"reading nan"<<endl;
                ros::spinOnce();
                lRate.sleep();
            }
             }



            //global position of object in /odom
            Ox=x - Oly*cos(th)-Olx*sin(th);
            Oy=y + Oly*sin(th)-Olx*cos(th);
            Oth=th - Olth;

            vOlx.push_back(Olx);
            vOly.push_back(Oly);
            vOlth.push_back(Olth);

            if(i>3){
             bool lost=1;
            for (int k=1;k<4;k++){

              //  cout<<"testing object lost i-k: "<<i-k<<" x(i-k): "<<vOlx[i-k]<<" x: "<<Olx<<" y(i-k): "<<vOly[i-k]<<" x: "<<Oly<<  endl;
                if ((vOlx[i-k]!=Olx)||(vOly[i-k]!=Oly)||(vOlth[i-k]!=Olth)) lost=0;

            }
            object_lost=lost;
            if(lost){
                robotino.singleMove(0, 0, 0, 0, 0, 0);
                lRate.sleep();
                ros:spinOnce();

            }

            }

            if (!object_lost){


            //calculating errors

            if (i==p-1){
                cout<<"recalculating plan"<<endl;
                int pom=p-i;
                double relx, rely;
                relx=100; rely=100;
                for(int j=0; j<p-1; j++)
                {
                   // cout<<"tren "<<sqrt(relx*relx+rely*rely)<<endl;
                    //cout<<"moguca "<<sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy))<<endl;
                        if (sqrt(relx*relx+rely*rely)>sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy)))
                        {
                            relx=X[j]-Ox;
                            rely=Y[j]-Oy;
                            pom=j;
                        }
                }
                i=pom;
            }


            errTh=TH[i]-Oth;
            errX=X[i]-Ox;
            errY=Y[i]-Oy;
           // {cout<<"current errors x: "<<errX<<" y:" <<errY<<" errTh: "<<errTh<<endl;}



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

            oFileR << "\t" << x << "\t" << y << "\t" <<yaw<<endl;
            oFileO << "\t" <<Olx << "\t" <<Oly << "\t" <<Olth<<endl;
            oFileER << "\t" << errX << "\t" << errY << "\t" <<errTh<<endl;


            if(sgn(Olx)*Olx>0.10){
               cout<<"turn Olx: "<<Olx<< endl;
               temp=0;
               while(temp<3.2){
                   //  cout<< "here 2"<<endl;
                   ros::spinOnce();
                   //  cout<< "here 3"<<endl;
                   if(sgn(Olx)>0) robotino.singleMove(-1*Oly/2*cos(temp), -1*Olx/2*sin(temp), 0, 0, 0, 0.1); //udesno i unazad
                 else robotino.singleMove( -1*Oly/2*cos(temp), Olx/2*sin(temp), 0, 0, 0, -0.1);

                  // cout <<"cos "<<cos(temp)<<" sin "<<sin(temp)<<endl;
                   temp=temp+deltat;
                   lRate.sleep();

               }

           }
           else if(sgn(Olx)*Olx>0.04){
                cout<<"here 1"<<endl;
                robotino.singleMove( 0, -1*Olx, 0, 0, 0, 0);
                i=i-1;
                 oFileVR << "\t" << 0 << "\t" << -1*Olx<< "\t" <<omega<<endl;

            }
            else if (sgn(errTh)*errTh> 0.5){
                cout<<"here 2"<<endl;
                robotino.singleMove( 0, 0, 0, 0, 0, omega);
                 oFileVR << "\t" << 0 << "\t" << 0 << "\t" <<omega<<endl;
              //  cout<<"here 1"<<endl;
            }
            else if (sgn(errY)*errY> 0.05){
                cout<<"here 3"<<endl;
               //if(vx>0.2)vx=0.2;
               //if(vy>0.2)vy=0.2;
                robotino.singleMove( vx, vy, 0, 0, 0, 0);
                 oFileVR << "\t" << vx << "\t" << vy << "\t" <<0<<endl;
               // cout<<"here 2"<<endl;
            }
           /* else if (errX<-0.05)
            {
                while(temp<6.4){
                    //  cout<< "here 2"<<endl;
                    ros::spinOnce();
                    //  cout<< "here 3"<<endl;
                    if(sgn(Olx)>0) robotino.singleMove(-1*Olx*cos(temp), -1*Olx*sin(temp), 0, 0, 0, 0.1); //udesno i unazad
                  else robotino.singleMove( -1*Olx*cos(temp), Olx*sin(temp), 0, 0, 0, -0.1);

                   // cout <<"cos "<<cos(temp)<<" sin "<<sin(temp)<<endl;
                    temp=temp+deltat;
                    lRate.sleep();

                }
            }*/
                else
            {
                cout<<"here 4"<<endl;
               //if(vx>0.2)vx=0.2;
              // if( vx>0){ //da ne ide unazad
             robotino.singleMove( vx, 0, 0, 0, 0, 0);
             oFileVR << "\t" <<vx << "\t" << 0 << "\t" <<0<<endl;
              // }
            }

           if (i<p-1)i++;
           else
           {cout<<"end"<<endl<<"current errors x: "<<errX<<" y:" <<errY<<endl;}
            lRate.sleep();

          }
         }

         break;
          }

     case 8:{
         double secs =ros::Time::now().toSec();
         while(1){
         ros::spinOnce();
         Odometry = robotino.getOdom();


        Olx=t.transform.translation.x -offsetX1;
        Oly=t.transform.translation.y -offsetY1;
         th=tf::getYaw(Odometry.pose.pose.orientation);

         Ox=x - Oly*cos(th)-Olx*sin(th);
         Oy=y + Oly*sin(th)-Olx*cos(th);
        // Olth=tf::getYaw(t.transform.rotation) -offsetTh;



         double aR2O=atan2(Oy-y,Ox-x);

         cout<<"time: "<<ros::Time::now().toSec()-secs<<endl;

        // cout<<" ugao "<<aR2O<<"      x"<< Ox<< "       y "<<Oy<<endl;


      //  cout<<"angle "<<tf::getYaw(Odometry.pose.pose.orientation)<<endl;

      //  cout<<"TAG: x: " <<t.transform.translation.x<<" y: "<<t.transform.translation.y<<" z: "<<t.transform.translation.z<<" yaw: "<<tf::getYaw(rotation)<<endl<<endl;
        //  robotino.singleMove( vx, vy, 0, 0, 0, -0.1);
         lRate.sleep();


         }
         break;
     }
     case 9:
          {



         x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;

         sleep(2);

         if( rO.call(R))
         {cout<<"call success"<<endl;}
         else{ cout<<"call not successful"<<endl; }
         sleep(5);

         cout<<"PUT THE OBJECT IN FRONT OF THE ROBOT"<<endl<<endl;

         //reading from file
         ifstream inFile;
         string filei="/home/c7031098/testing/robotino/"+tr;
         inFile.open (filei.c_str(),ios::in);
         string val;

         string pathR="/home/c7031098/testing/robotino/pathR"+expN;
         ofstream oFileR;
         oFileR.open(pathR.c_str());

         string errR="/home/c7031098/testing/robotino/errR"+expN;
         ofstream oFileER;
         oFileER.open(errR.c_str());

         string velR="/home/c7031098/testing/robotino/velR"+expN;
         ofstream oFileVR;
         oFileVR.open(velR.c_str());

         string pathO="/home/c7031098/testing/robotino/pathO"+expN;
         ofstream oFileO;
         oFileO.open(pathO.c_str());

         cout<<"file open"<<endl;

         double n = 0;

         while(!inFile.eof()) {

             for(int i = 0; i < 3; ++i) {
                inFile>>val;
                 n = string_to_double(val);
                 if(i == 0) { X.push_back(n); }
                 else if(i == 1) { Y.push_back(n); }
                 else { TH.push_back(n); }
             }
             fflush(stdout);
         }



         rotation = t.transform.rotation;
         Xo1 =t.transform.translation.x; Yo1=t.transform.translation.y; Tho1=tf::getYaw(rotation);

         //path tracking
         vector<double> Ex,Ey,Eth, dEx,dEy,dEth,vOlx,vOly,vOlth;

         double vx,vy,omega,derrX,derrY,derrTh;
         int p=X.size();
         int i=0;
        // while ((i<p-1)||((i==p-1)&&((abs(X[p-1]-Ox)>0.05)||(abs(Y[p-1]-Oy)>0.05)))){ //error tolerance of 5 cm


         X[i]=0;
         Y[i]=-0.30;

             ros::spinOnce();
             //robotino position in /odom world
             Odometry = robotino.getOdom();
             x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;
             th=tf::getYaw(Odometry.pose.pose.orientation);

             //relative position of the object
             int check=0;
             while(!check){
            Olx=t.transform.translation.x -offsetX1;
            Oly=t.transform.translation.y -offsetY1;

            Olth=tf::getYaw(t.transform.rotation) -offsetTh1;
            if(!isnan(Olth))check=1;
            else {
                cout<<"reading nan"<<endl;
                ros::spinOnce();
                lRate.sleep();
            }
             }



            //global position of object in /odom
            Ox=x - Oly*cos(th)-Olx*sin(th);
            Oy=y + Oly*sin(th)-Olx*cos(th);
            Oth=th - Olth;

            vOlx.push_back(Olx);
            vOly.push_back(Oly);
            vOlth.push_back(Olth);

            if(i>3){
             bool lost=1;
            for (int k=1;k<4;k++){

              //  cout<<"testing object lost i-k: "<<i-k<<" x(i-k): "<<vOlx[i-k]<<" x: "<<Olx<<" y(i-k): "<<vOly[i-k]<<" x: "<<Oly<<  endl;
                if ((vOlx[i-k]!=Olx)||(vOly[i-k]!=Oly)||(vOlth[i-k]!=Olth)) lost=0;

            }
            object_lost=lost;
            if(lost){
                robotino.singleMove(0, 0, 0, 0, 0, 0);
                lRate.sleep();
                ros::spinOnce();

            }

            }

            if (!object_lost){


            //calculating errors


                // reaching goal adaptation
            if (i==p-1){
                cout<<"recalculating plan"<<endl;
                int pom=p-i;
                double relx, rely;
                relx=100; rely=100;
                for(int j=0; j<p-1; j++)
                {
                   // cout<<"tren "<<sqrt(relx*relx+rely*rely)<<endl;
                    //cout<<"moguca "<<sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy))<<endl;
                        if (sqrt(relx*relx+rely*rely)>sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy)))
                        {
                            relx=X[j]-Ox;
                            rely=Y[j]-Oy;
                            pom=j;
                        }
                }
                i=pom;
            }



            //approach witouth observing object relative orintation


            errTh=TH[i]-Oth;
            errX=X[i]-Ox;
            errY=Y[i]-Oy;
           // {cout<<"current errors x: "<<errX<<" y:" <<errY<<" errTh: "<<errTh<<endl;}


            //robot has to be on the line created from point from path and object
                                                                       //to push object towards path

            double aO2P=atan2(errY,errX); //angle which form object and desired path  with X

            double aR2P=atan2(Y[i]-y,X[i]-x); //angle which form robot and desired path  with X

            double aR2O=atan2(Oy-y,Ox-x);  //angle robot to object -> inverse formula because of robot coordinate system
             if(aR2O>3.14)aR2O=aR2O-2*3.14;
            if(aR2O<-3.14)aR2O=aR2O+2*3.14;

            cout<<" x:"<<Ox<<"y: "<<Oy<<endl;


            //checking distance of robot to the path and  robot to the object
            //distance between robot and path has to be longer then robot to the object

            double dR2O= sqrt((x-Ox)*(x-Ox)+(y-Oy)*(y-Oy)); //d(robot, object)
            double dR2P= sqrt((x-X[i])*(x-X[i])+(y-Y[i])*(y-Y[i]));  //d(robot, path point)
            double dO2P= sqrt(errX*errX+errY*errY);

            cout<<"distances: r-o "<< dR2O<<" r-p "<< dR2P <<" o-p "<<dO2P<<endl;

            double aRPO=acos((dR2P*dR2P+dO2P*dO2P-dR2O*dR2O)/(2*dO2P*dR2P));  //angle which is formed by this triangle with center in P
            if (abs(aRPO)>3.14) aRPO=2*3.14-aRPO;

            cout<<"angle: "<<aRPO<<endl;

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

            Tho=tf::getYaw(rotation);

            oFileR << "\t" << x << "\t" << y << "\t" <<yaw<<endl;
            oFileO << "\t" <<Olx << "\t" <<Oly << "\t" <<Olth<<endl;
            oFileER << "\t" << errX << "\t" << errY << "\t" <<errTh<<endl;


            //if robot is in not in good position compared with object and path
           if ((dR2P-dO2P)<0){
            //executing rotational motion to place robot on right direction

                //turning robot to face the object
                double errTH=th-aR2O;
                cout<<th<<"  "<<aR2O<<endl;

                //if(errTH>3.14)errTH=errTH-2*3.14;
                //if(errTH<-3.14)errTH=errTH+2*3.14;
                cout<<"error of robot orientation "<< errTH<<endl;

                while(abs(errTH)>0.01){
                    ros::spinOnce();
                    //robotino position in /odom world
                    Odometry = robotino.getOdom();
                    th=tf::getYaw(Odometry.pose.pose.orientation);
                  //  cout<<"current agle of the srobot"<< errTH<<endl;
                    if(errTH>0.02)robotino.singleMove( 0, 0, 0, 0, 0, -0.1);
                    if(errTH<-0.02)robotino.singleMove( 0, 0, 0, 0, 0, 0.1);
                    errTH=th-aR2O;
                    // cout<<th<<"  "<<aR2O<<endl;

                   // cout<<"error of robot orientation "<< errTH<<endl;

                    lRate.sleep();

                }

                cout<<"orientation done"<<endl;
                sleep(3);
                double v_circ;
                if(dR2O<0.10){

                    for(int ic=0;ic<5;ic++){
                        robotino.singleMove(-0.2,0,0,0,0,0);
                        ros::spinOnce();
                        lRate.sleep();
                    }

                    //calculating positions again->(MAKE FUNCTION OF IT)
                    ros::spinOnce();
                    Odometry = robotino.getOdom();
                    x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;
                    th=tf::getYaw(Odometry.pose.pose.orientation);

                    //relative position of the object
                    int check=0;
                    while(!check){
                   Olx=t.transform.translation.x -offsetX1;
                   Oly=t.transform.translation.y -offsetY1;
                   Olth=tf::getYaw(t.transform.rotation) -offsetTh1;
                   if(!isnan(Olth))check=1;
                   else {
                       cout<<"reading nan"<<endl;
                       ros::spinOnce();
                       lRate.sleep();
                   }
                    }

                   //global position of object in /odom
                   Ox=x - Oly*cos(th)-Olx*sin(th);
                   Oy=y + Oly*sin(th)-Olx*cos(th);
                   Oth=th - Olth;

                   dR2O= sqrt((x-Ox)*(x-Ox)+(y-Oy)*(y-Oy)); //d(robot, object)
                   dR2P= sqrt((x-X[i])*(x-X[i])+(y-Y[i])*(y-Y[i]));  //d(robot, path point)
                   dO2P= sqrt(errX*errX+errY*errY);

                   aRPO=acos((dR2P*dR2P+dO2P*dO2P-dR2O*dR2O)/(2*dO2P*dR2P));  //angle which is formed by this triangle with center in P
                   if (abs(aRPO)>3.14) aRPO=2*3.14-aRPO;

                   cout<<"angle: "<<aRPO<<endl;
                }
                if(dR2O>0.4){
                     for(int ic=0;ic<10;ic++){
                        robotino.singleMove(0.2,0,0,0,0,0);
                        ros::spinOnce();
                        lRate.sleep();
                     }

                     ros::spinOnce();
                     Odometry = robotino.getOdom();
                     x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;
                     th=tf::getYaw(Odometry.pose.pose.orientation);

                     //relative position of the object
                     int check=0;
                     while(!check){
                    Olx=t.transform.translation.x -offsetX1;
                    Oly=t.transform.translation.y -offsetY1;
                    Olth=tf::getYaw(t.transform.rotation) -offsetTh1;
                    if(!isnan(Olth))check=1;
                    else {
                        cout<<"reading nan"<<endl;
                        ros::spinOnce();
                        lRate.sleep();
                    }
                     }

                    //global position of object in /odom
                    Ox=x - Oly*cos(th)-Olx*sin(th);
                    Oy=y + Oly*sin(th)-Olx*cos(th);
                    Oth=th - Olth;

                    dR2O= sqrt((x-Ox)*(x-Ox)+(y-Oy)*(y-Oy)); //d(robot, object)
                    dR2P= sqrt((x-X[i])*(x-X[i])+(y-Y[i])*(y-Y[i]));  //d(robot, path point)
                    dO2P= sqrt(errX*errX+errY*errY);

                    aRPO=acos((dR2P*dR2P+dO2P*dO2P-dR2O*dR2O)/(2*dO2P*dR2P));  //angle which is formed by this triangle with center in P
                    if (abs(aRPO)>3.14) aRPO=2*3.14-aRPO;

                    cout<<"angle: "<<aRPO<<endl;
                }


                if (dR2O<=0.4) v_circ=1.2*dR2O;
                else
                {
                    v_circ=0.4;
                }

                cout<<"v_cric is "<<v_circ<<endl;

                double start =ros::Time::now().toSec();
                while(abs(aRPO)>0.1)
                {
                   // while(1){

                    ros::spinOnce();
                    Odometry = robotino.getOdom();
                    //  cout<< "here 3"<<endl;
                  //if(sgn(Ox)>0) robotino.singleMove(2*dR2O*sin(temp), 2*dR2O*cos(temp), 0, 0, 0, 0); //udesno i unazad
                  //else
                    temp=ros::Time::now().toSec()-start;
                   // cout<<temp<<endl;

                    robotino.singleMove( v_circ*sin(temp), v_circ*cos(temp), 0, 0, 0, 0);

                  // cout <<"cos "<<cos(temp)<<" sin "<<sin(temp)<<endl;
                  x = Odometry.pose.pose.position.x  ; //getting new position of robot (it is assumed that object is not moving)
                  y = Odometry.pose.pose.position.y ;

                   // temp=temp+deltat;

                    dR2O= sqrt((x-Ox)*(x-Ox)+(y-Oy)*(y-Oy)); //d(robot, object)
                    dR2P= sqrt((x-X[i])*(x-X[i])+(y-Y[i])*(y-Y[i]));  //d(robot, path point)
                    dO2P= sqrt(errX*errX+errY*errY);

                    aRPO=acos((dR2P*dR2P+dO2P*dO2P-dR2O*dR2O)/(2*dO2P*dR2P));  //angle which is formed by this triangle with center in P
                    if (abs(aRPO)>3.14) aRPO=2*3.14-aRPO;
                    lRate.sleep();

                }

           //S }


/*

            vx=(0.4*(errX*cos(th)+errY*sin(th))+0.1*(derrX*cos(th)+derrY*sin(th)));//*sgn(errX*cos(th)+errY*sin(th))+0.1*(derrX*cos(th)+derrY*sin(th));
            vy=0.2*(errY*cos(th)-errX*sin(th))+0.1*(derrY*cos(th)-derrX*sin(th));
            omega=0.5*errTh;







            if(sgn(Olx)*Olx>0.10){
               cout<<"turn Olx: "<<Olx<< endl;
               temp=0;
               while(temp<3.2){
                   //  cout<< "here 2"<<endl;
                   ros::spinOnce();
                   //  cout<< "here 3"<<endl;
                   if(sgn(Olx)>0) robotino.singleMove(-1*Oly/2*cos(temp), -1*Olx/2*sin(temp), 0, 0, 0, 0.1); //udesno i unazad
                 else robotino.singleMove( -1*Oly/2*cos(temp), Olx/2*sin(temp), 0, 0, 0, -0.1);

                  // cout <<"cos "<<cos(temp)<<" sin "<<sin(temp)<<endl;
                   temp=temp+deltat;
                   lRate.sleep();

               }

           }
           else if(sgn(Olx)*Olx>0.04){
                cout<<"here 1"<<endl;
                robotino.singleMove( 0, -1*Olx, 0, 0, 0, 0);
                i=i-1;
                 oFileVR << "\t" << 0 << "\t" << -1*Olx<< "\t" <<omega<<endl;

            }
            else if (sgn(errTh)*errTh> 0.5){
                cout<<"here 2"<<endl;
                robotino.singleMove( 0, 0, 0, 0, 0, omega);
                 oFileVR << "\t" << 0 << "\t" << 0 << "\t" <<omega<<endl;
              //  cout<<"here 1"<<endl;
            }
            else if (sgn(errY)*errY> 0.05){
                cout<<"here 3"<<endl;
               //if(vx>0.2)vx=0.2;
               //if(vy>0.2)vy=0.2;
                robotino.singleMove( vx, vy, 0, 0, 0, 0);
                 oFileVR << "\t" << vx << "\t" << vy << "\t" <<0<<endl;
               // cout<<"here 2"<<endl;
            }
           /* else if (errX<-0.05)
            {
                while(temp<6.4){
                    //  cout<< "here 2"<<endl;
                    ros::spinOnce();
                    //  cout<< "here 3"<<endl;
                    if(sgn(Olx)>0) robotino.singleMove(-1*Olx*cos(temp), -1*Olx*sin(temp), 0, 0, 0, 0.1); //udesno i unazad
                  else robotino.singleMove( -1*Olx*cos(temp), Olx*sin(temp), 0, 0, 0, -0.1);

                   // cout <<"cos "<<cos(temp)<<" sin "<<sin(temp)<<endl;
                    temp=temp+deltat;
                    lRate.sleep();

                }
            }*/
          /*      else
            {
                cout<<"here 4"<<endl;
               //if(vx>0.2)vx=0.2;
              // if( vx>0){ //da ne ide unazad
             robotino.singleMove( vx, 0, 0, 0, 0, 0);
             oFileVR << "\t" <<vx << "\t" << 0 << "\t" <<0<<endl;
              // }
            }

           if (i<p-1)i++;
           else
           {cout<<"end"<<endl<<"current errors x: "<<errX<<" y:" <<errY<<endl;}
            lRate.sleep();

          } */
         }
         }

         break;
          }

     case 10: {

     double e1=0.1;
     double e2=0.1;
     double t=0;

     double deltat=1e-1;
     double start =ros::Time::now().toSec();

    //cout<< "here 1"<<endl;

     while(t<6.4){


         //  cout<< "here 2"<<endl;
         ros::spinOnce();
         t=ros::Time::now().toSec()-start;
         //  cout<< "here 3"<<endl;
      // robotino.singleMove(-1* 0.4*cos(t), 0.2*sin(t), 0, 0, 0, 0); //udesno i unazad
         robotino.singleMove( 0.4*sin(t), 0.4*cos(t), 0, 0, 0, 0);

         cout <<"cos "<<cos(t)<<" sin "<<sin(t)<<endl;

         lRate.sleep();

     }
     break;
     }
      case 11: {



         sleep(2);

         cout<<"Offest for x 1 is: "<<offsetX1<<endl;
         cout<<"Offset for y 1 is: "<<offsetY1<<endl;
         cout<<"Offset for th  1 is: "<<offsetTh1<<endl<<endl;



         //lines to be change by integration
         //--------------------------------------------------------------------------------------------------------------

         //reseting odometry -- not necessary

         if( rO.call(R))
         {cout<<"odometry reset done"<<endl;}
         else{ cout<<"odometry reset call not successful"<<endl; }

         sleep(5);



         //reading from file and writing to files
         ifstream inFile;
         string filei="/home/c7031098/testing/robotino/"+tr;
         inFile.open (filei.c_str(),ios::in);
         string val;

         string pathR="/home/c7031098/testing/robotino/pathR"+expN;
         ofstream oFileR;
         oFileR.open(pathR.c_str());

         string pathP="/home/c7031098/testing/robotino/pathP"+expN;
         ofstream oFileP;
         oFileP.open(pathP.c_str());

         string errR="/home/c7031098/testing/robotino/errR"+expN;
         ofstream oFileER;
         oFileER.open(errR.c_str());

         string velR="/home/c7031098/testing/robotino/velR"+expN;
         ofstream oFileVR;
         oFileVR.open(velR.c_str());

         string pathO="/home/c7031098/testing/robotino/pathO"+expN;
         ofstream oFileO;
         oFileO.open(pathO.c_str());

         cout<<"files open"<<endl;

         //----------------------------------------------------------------------------------------
         //geting wanted path from file ->to be changed

         double n = 0;

         while(!inFile.eof()) {

             for(int i = 0; i < 3; ++i) {
                 inFile>>val;
                 n = string_to_double(val);
                 if(i == 0) { X.push_back(n+d); }
                 else if(i == 1) { Y.push_back(n); }
                 else { TH.push_back(n); }
             }
             fflush(stdout);
         }

         /*for(int i = 0; i < 3; ++i) {
             X[i]= X[i]+d+0.01;

         }*/

        //----------------------------------------------------------------------------------------



         //path tracking

         //variables

         vector<double> Ex,Ey,Eth, dEx,dEy,dEth,vOlx,vOly,vOlth;

         double vx,vy,omega,derrX,derrY,derrTh;

         double dR2O, dR2P, dO2P, aRPO, aR2O;
         int p=X.size();
         int i=0;
         bool lost;



       //  X[i]=0;
         //Y[i]=0.30;


         ros::spinOnce();

         update_robot_coordiantes(&robotino);
         update_object_coordiantes();

         //cout<<"robot "<<x<<"   "<<y<<"  "<<th<<"   "<<endl;
         //cout<<"object "<<Olx<<"   "<<Oly<<"  "<<Olth<<"   "<<endl;

         //control starting
         while ((i<p-1)||((i==p-1)&&((abs(X[p-1]-Ox)>0.05)||(abs(Y[p-1]-Oy)>0.05)))){ //error tolerance of 5 cm

             ros::spinOnce();


             lost=0;
             update_robot_coordiantes(&robotino);
             lost=!update_object_coordiantes();

             cout<<endl<<"object l "<<Olx<<"   "<<Oly<<"  "<<Olth<<"   "<<endl;
             cout<<endl<<"robot "<<x<<"   "<<y<<"  "<<th<<"   "<<endl;
             cout<<"object "<<Ox<<"   "<<Oy<<"  "<<Oth<<"   "<<endl;
             cout<<"wanted "<<X[i]<<"   "<<Y[i]<<"  "<<TH[i]<<"   "<<endl;

             vOlx.push_back(Olx); //local object coordinates
             vOly.push_back(Oly);
             vOlth.push_back(Olth);

              bool lost1;
             if(i>10){
                 lost1=1;
             for (int k=1;k<10;k++){

               //  cout<<"testing object lost i-k: "<<i-k<<" x(i-k): "<<vOlx[i-k]<<" x: "<<Olx<<" y(i-k): "<<vOly[i-k]<<" x: "<<Oly<<  endl;
                 if ((vOlx[i-k]!=Olx)||(vOly[i-k]!=Oly)||(vOlth[i-k]!=Olth)) lost1=0;

             }
             }


            if (lost1) {
                cout <<"object lost"<<endl; //to replace with mesage for planner
                //rotational movement of robot for object search- to implement
                break;
            }


            //recalculating plan if end of control sequence reached
            //finding the closest point i path
            if (i==p-1){
                cout<<"recalculating plan"<<endl;
                int pom=p-i;
                double relx, rely;
                relx=100; rely=100;
                for(int j=0; j<p-1; j++){
                    if (sqrt(relx*relx+rely*rely)>sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy))){
                            relx=X[j]-Ox;
                            rely=Y[j]-Oy;
                            pom=j;
                        }
                }
                i=pom;
            }

            //----------------------------------------------------------

            //calculating errors of objects compared with wanted ones
            errTh=TH[i]-Oth;
            errX=X[i]-Ox;
            errY=Y[i]-Oy;
            {cout<<"current errors x: "<<errX<<" y:" <<errY<<" errTh: "<<errTh<<endl;}

            Ex.push_back(errX);
            Ey.push_back(errY);
            Eth.push_back(errTh);


            if(i==1){
                derrX=0; derrY=0; derrTh=0;
            }
            else{
                derrX=errX-Ex[i-1];//derivatives ( T is included in gain value)
                derrY=errY-Ey[i-1];
                derrTh=errTh-Eth[i-1];
            }


            dEx.push_back(derrX);
            dEy.push_back(derrY);
            dEth.push_back(derrTh);


            //checking distance of robot to the path and  robot to the object
            //distance between robot and path has to be longer then robot to the object

             dR2O= sqrt((x-Ox)*(x-Ox)+(y-Oy)*(y-Oy)); //d(robot, object)
             dR2P= sqrt((x-X[i])*(x-X[i])+(y-Y[i])*(y-Y[i]));  //d(robot, path point)
             dO2P= sqrt(errX*errX+errY*errY);

           // cout<<"distances: r-o "<< dR2O<<" r-p "<< dR2P <<" o-p "<<dO2P<<endl;

            aRPO=acos((dR2P*dR2P+dO2P*dO2P-dR2O*dR2O)/(2*dO2P*dR2P));  //angle which is formed by this triangle with center in P
            if (aRPO>3.14) aRPO=aRPO-2*3.14;
            if (aRPO<-3.14) aRPO=aRPO+2*3.14;

            aR2O=atan2(Oy-y,Ox-x);  //angle robot to object -> inverse formula because of robot coordinate system
            if(aR2O>3.14)aR2O=aR2O-2*3.14;
            if(aR2O<-3.14)aR2O=aR2O+2*3.14;

            //calculating local position of point from path
            double Plx=-(X[i]-x)*sin(th)-(Y[i]-y)*cos(th);
            double Ply=(X[i]-x)*cos(th)-(Y[i]-y)*cos(th);

            //calling control actions

            //action 1 : correcting robot- object orientation
         if(abs(aR2O)>0.4){
                cout<<"action 1 "<<endl;
                cout<<"angle"<<aR2O<<endl;
                face_object(&robotino,aR2O);
                i=i-1;}

            //action 2 : correcting robot-object position
        /*  else if ((abs(Olx)>0.10)|| (Oly<-(d+0.15))){
                 cout<<"action 2 "<<endl;
                 robotino.singleMove( 0.5*(abs(Oly)-d), -1*Olx, 0, 0, 0, omega);
                i=i-1;

            }*/

         else if (((Plx-Olx)>0.05)||((dR2P-dO2P)<-0.2)){

             cout<<"action new"<<endl;

             if (Olx<0) robotino.singleMove(0, Plx-Olx, 0, 0, 0, 0);//turn left with respect to object
             else robotino.singleMove( 0, -(Plx-Olx), 0, 0, 0, 0);//turn right  with respect to object



             /*double start =ros::Time::now().toSec();
             double temp;
             double v_circ=dR2O+d;


             double turn=Olx-Plx; //object error in robot world
             cout<<"turn "<<turn<<endl;

            /* while((Plx-Olx)>0.05){

                 temp=ros::Time::now().toSec()-start;
                // face_object(&robotino,aR2O);
                 update_robot_coordiantes(&robotino);
                 update_object_coordiantes();

                 Plx=-(X[i]-x)*sin(th)-(Y[i]-y)*cos(th);
                 Ply=(X[i]-x)*cos(th)-(Y[i]-y)*cos(th);

                 if (Olx<0) robotino.singleMove( v_circ*sin(temp), v_circ*cos(temp), 0, 0, 0, 0);//turn left with respect to object
                 else robotino.singleMove( v_circ*sin(temp), -v_circ*cos(temp), 0, 0, 0, 0);//turn right  with respect to object

                 ros::spinOnce();
                 lRate.sleep();

             }
            /* update_robot_coordiantes(&robotino);
             lost=0;
             lost=!update_object_coordiantes();
             while(lost){
                 if (turn>0) robotino.singleMove( 0, 0, 0, 0, 0, 0.1);
                 else robotino.singleMove( 0, 0, 0, 0, 0, -0.1);
                 lost=!update_object_coordiantes();
             }*/
             //face_object(&robotino,aR2O);


         }

         //action 2 : correcting robot-object position
         else if ((abs(Olx)>0.10)|| (Oly<-(d+0.15))){
              cout<<"action 2 "<<endl;
              robotino.singleMove( 0.5*(abs(Oly)-d), -1*Olx, 0, 0, 0, omega);
             i=i-1;

         }
            //action 3 : correct robot position compared with object observing the angle of pushing

         /*else if (((dR2P-dO2P)<-0.2)||(aRPO>0.2)){
          // else if ((dR2P-dO2P)<-0.21){

            cout<<"R2P "<<dR2P<<" O2P " <<dO2P<<" ugao "<<aRPO<<endl;

           // cout<<"action  3"<<endl;
           // sleep(5);

            face_object(&robotino,aR2O);


            double start =ros::Time::now().toSec();
            double temp;
            double v_circ=dR2O+d;



            double turn=Olx-Plx; //object error in robot world

            while(abs(aRPO)>0.1){


                temp=ros::Time::now().toSec()-start;

                update_robot_coordiantes(&robotino);
                dR2O= sqrt((x-Ox)*(x-Ox)+(y-Oy)*(y-Oy)); //d(robot, object)
                dR2P= sqrt((x-X[i])*(x-X[i])+(y-Y[i])*(y-Y[i]));  //d(robot, path point)
                dO2P= sqrt(errX*errX+errY*errY);

                aRPO=acos((dR2P*dR2P+dO2P*dO2P-dR2O*dR2O)/(2*dO2P*dR2P));  //angle which is formed by this triangle with center in P
               // cout<<(dR2P*dR2P+dO2P*dO2P-dR2O*dR2O)/(2*dO2P*dR2P)<<endl;
                if (aRPO>3.14) aRPO=aRPO-2*3.14;
                if (aRPO<-3.14) aRPO=aRPO+2*3.14;
                if (turn<0) robotino.singleMove( v_circ*sin(temp), v_circ*cos(temp), 0, 0, 0, 0);//turn left with respect to object
                else robotino.singleMove( v_circ*sin(temp), -v_circ*cos(temp), 0, 0, 0, 0);//turn right  with respect to object
               // cout<<"angle "<<aRPO<< " "<< dR2P<< " "<<dO2P<< endl;
                cout<<"turn "<<turn<<endl;
                ros::spinOnce();
                lRate.sleep();

            }
            update_robot_coordiantes(&robotino);
            lost=0;
            lost=!update_object_coordiantes();
            while(lost){
                if (turn>0) robotino.singleMove( 0, 0, 0, 0, 0, 0.1);
                else robotino.singleMove( 0, 0, 0, 0, 0, -0.1);
                lost=!update_object_coordiantes();
            }
            face_object(&robotino,aR2O);

           // i=i-1;
            }*/
            //action 4: path tracking controler
            else{
                cout<<"action 4"<<endl;

                vx=(0.4*(errX*cos(th)+errY*sin(th))+0.1*(derrX*cos(th)+derrY*sin(th)));
                vy=0.2*(errY*cos(th)-errX*sin(th))+0.1*(derrY*cos(th)-derrX*sin(th));
                 cout<<"vx "<<vx<<" vy "<<vy<<endl;
                robotino.singleMove( vx, vy, 0, 0, 0, 0);
            }


          cout<<"action 4"<<endl;

        /*  vx=0.4*(errX*cos(th)+errY*sin(th))+0.1*(derrX*cos(th)+derrY*sin(th));
          vy=0.2*(errY*cos(th)-errX*sin(th))+0.1*(derrY*cos(th)-derrX*sin(th));
           cout<<"vx "<<vx<<" vy "<<vy<<endl;
          robotino.singleMove( vx, vy, 0, 0, 0, 0);*/

           if (i<p-1)i++;
           else{
            cout<<"Target "<<endl<<"current errors x: "<<errX<<" y:" <<errY<<endl;}
            lRate.sleep();


          }


         break;
          }
     case 12 : {

         cout<<"Offest for x 1 is: "<<offsetX1<<endl;
         cout<<"Offset for y 1 is: "<<offsetY1<<endl;
         cout<<"Offset for th  1 is: "<<offsetTh1<<endl<<endl;

         string traj="/home/c7031098/testing/robotino/"+tr;
         ofstream oFile;
         oFile.open(traj.c_str());
         cout<<"file open"<<endl;

         //performing desired path
        while(x<initX+0.4){

            ros::spinOnce();
            update_robot_coordiantes(&robotino);
            update_object_coordiantes();
            oFile << "\t" << Ox << "\t" << Oy << "\t" <<Oth<<endl;

             robotino.singleMove( 0.05, 0, 0, 0, 0, 0);
           // cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<" yaw "<<yaw<< endl<<endl<<endl;
            lRate.sleep();

       }

        cout<<"phase 1 done"<<endl;
        while(th>initth-1.50){

            ros::spinOnce();

            update_robot_coordiantes(&robotino);
            update_object_coordiantes();
            oFile << "\t" << Ox << "\t" << Oy << "\t" <<Oth<<endl;

             robotino.singleMove( 0, 0, 0, 0, 0, -0.05);

              oFile << "\t" << x << "\t" << y << "\t" <<yaw<<endl;

           // cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<" yaw "<<yaw<< endl<<endl<<endl;

            //cout<<"current pos x:"<<x<<" y: "<<y<<" z: "<<z<<endl<<endl<<endl;
            lRate.sleep();

       }

         cout<<"phase 2 done"<<endl;

       // cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<endl<<endl<<endl<<endl<<endl;

        while(y>-0.4){

            ros::spinOnce();

            update_robot_coordiantes(&robotino);
            update_object_coordiantes();
            oFile << "\t" << Ox << "\t" << Oy << "\t" <<Oth<<endl;

             robotino.singleMove( 0.05, 0, 0, 0, 0, 0);

             oFile << "\t" << x << "\t" << y << "\t" <<yaw<<endl;

           // cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<" yaw "<<yaw<< endl<<endl<<endl;

            //cout<<"current pos x:"<<x<<" y: "<<y<<" th: "<<th<<endl;
            lRate.sleep();
             cout<<"phase 3 done"<<endl;

       }

         break;

     }

     case 13:
          {



         if( rO.call(R))
         {cout<<"call success"<<endl;}
         else{ cout<<"call not successful"<<endl; }
         sleep(5);

         cout<<"PUT THE OBJECT IN FRONT OF THE ROBOT"<<endl<<endl;

         //reading from file
         ifstream inFile;
         string filei="/home/c7031098/testing/robotino/"+tr;
         inFile.open (filei.c_str(),ios::in);
         string val;

         string pathR="/home/c7031098/testing/robotino/pathR"+expN;
         ofstream oFileR;
         oFileR.open(pathR.c_str());

         string errR="/home/c7031098/testing/robotino/errR"+expN;
         ofstream oFileER;
         oFileER.open(errR.c_str());

         string velR="/home/c7031098/testing/robotino/velR"+expN;
         ofstream oFileVR;
         oFileVR.open(velR.c_str());

         string pathO="/home/c7031098/testing/robotino/pathO"+expN;
         ofstream oFileO;
         oFileO.open(pathO.c_str());

         cout<<"file open"<<endl;

         double n = 0;

         while(!inFile.eof()) {

             for(int i = 0; i < 3; ++i) {
                inFile>>val;
                 n = string_to_double(val);
                 if(i == 0) { X.push_back(n+d); }
                 else if(i == 1) { Y.push_back(n); }
                 else { TH.push_back(n); }
             }
             fflush(stdout);
         }



         update_robot_coordiantes(&robotino);
         update_object_coordiantes();
         //path tracking
         vector<double> Ex,Ey,Eth, dEx,dEy,dEth,vOlx,vOly,vOlth;
          double dR2O, dR2P, dO2P, aRPO, aR2O,aORP;

         double vx,vy,omega,derrX,derrY,derrTh;
         int p=X.size();
         int i=0;
         while ((i<p-1)||((i==p-1)&&((abs(X[p-1]-Ox)>0.05)||(abs(Y[p-1]-Oy)>0.05)))){ //error tolerance of 5 cm

             ros::spinOnce();
             //robotino position in /odom world
             update_robot_coordiantes(&robotino);
             update_object_coordiantes();


             //checking distance of robot to the path and  robot to the object
             //distance between robot and path has to be longer then robot to the object

              dR2O= sqrt((x-Ox)*(x-Ox)+(y-Oy)*(y-Oy)); //d(robot, object)
              dR2P= sqrt((x-X[i])*(x-X[i])+(y-Y[i])*(y-Y[i]));  //d(robot, path point)
              dO2P= sqrt(errX*errX+errY*errY);

            // cout<<"distances: r-o "<< dR2O<<" r-p "<< dR2P <<" o-p "<<dO2P<<endl;

             aORP=acos((dR2P*dR2P-dO2P*dO2P+dR2O*dR2O)/(2*dR2O*dR2P));
             if (aORP>3.14) aORP=aORP-2*3.14;
             if (aORP<-3.14) aORP=aORP+2*3.14;

             aR2O=atan2(Oy-y,Ox-x);  //angle robot to object -> inverse formula because of robot coordinate system
             if(aR2O>3.14)aR2O=aR2O-2*3.14;
             if(aR2O<-3.14)aR2O=aR2O+2*3.14;

             //calculating local position of point from path
             double Plx=-(X[i]-x)*sin(th)-(Y[i]-y)*cos(th);
             double Ply=(X[i]-x)*cos(th)-(Y[i]-y)*cos(th);

            vOlx.push_back(Olx);
            vOly.push_back(Oly);
            vOlth.push_back(Olth);

            if(i>3){
                bool lost=1;
                for (int k=1;k<4;k++){

                    cout<<"testing object lost i-k: "<<i-k<<" x(i-k): "<<vOlx[i-k]<<" x: "<<Olx<<" y(i-k): "<<vOly[i-k]<<" x: "<<Oly<<  endl;
                    if ((vOlx[i-k]!=Olx)||(vOly[i-k]!=Oly)||(vOlth[i-k]!=Olth)) lost=0;

                }
                object_lost=lost;
                if(lost){
                    robotino.singleMove(0, 0, 0, 0, 0, 0);
                    lRate.sleep();
                   ros::spinOnce();
                   cout <<"object lost"<<endl; //to replace with mesage for planner
                   //rotational movement of robot for object search- to implement
                   break;

                }
            }



            //calculating errors

            if (i==p-1){
                cout<<"recalculating plan"<<endl;
                int pom=p-i;
                double relx, rely;
                relx=100; rely=100;
                for(int j=0; j<p-1; j++)
                {
                   // cout<<"tren "<<sqrt(relx*relx+rely*rely)<<endl;
                    //cout<<"moguca "<<sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy))<<endl;
                        if (sqrt(relx*relx+rely*rely)>sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy)))
                        {
                            relx=X[j]-Ox;
                            rely=Y[j]-Oy;
                            pom=j;
                        }
                }
                i=pom;
            }


            errTh=TH[i]-Oth;
            errX=X[i]-Ox;
            errY=Y[i]-Oy;
           // {cout<<"current errors x: "<<errX<<" y:" <<errY<<" errTh: "<<errTh<<endl;}



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

            oFileR << "\t" << x << "\t" << y << "\t" <<yaw<<endl;
            oFileO << "\t" <<Olx << "\t" <<Oly << "\t" <<Olth<<endl;
            oFileER << "\t" << errX << "\t" << errY << "\t" <<errTh<<endl;


           /* if(sgn(Olx)*Olx>0.10){
               cout<<"turn Olx: "<<Olx<< endl;
               temp=0;
               while(temp<3.2){
                   //  cout<< "here 2"<<endl;
                   ros::spinOnce();
                   //  cout<< "here 3"<<endl;
                   if(sgn(Olx)>0) robotino.singleMove(-1*Oly/2*cos(temp), -1*Olx/2*sin(temp), 0, 0, 0, 0.1); //udesno i unazad
                 else robotino.singleMove( -1*Oly/2*cos(temp), Olx/2*sin(temp), 0, 0, 0, -0.1);

                  // cout <<"cos "<<cos(temp)<<" sin "<<sin(temp)<<endl;
                   temp=temp+deltat;
                   lRate.sleep();

               }

           }*/
          if(sgn(Olx)*Olx>0.04){
                cout<<"here 1"<<endl;
                robotino.singleMove( 0, -1*Olx, 0, 0, 0, 0);
                i=i-1;
                 oFileVR << "\t" << 0 << "\t" << -1*Olx<< "\t" <<omega<<endl;

            }
           else if ((dR2P+0.05<dO2P+dR2O)&&(aORP>0.4)){
                // else if ((dR2P-dO2P)<-0.21){

                  cout<<"R2P "<<dR2P<<" O2P " <<dO2P<<" R2O " <<dR2O<<" ugao "<<aORP<<endl;


                 // cout<<"action  3"<<endl;
                 // sleep(5);

                  face_object(&robotino,aR2O);


                  double start =ros::Time::now().toSec();
                  double temp;
                  double v_circ=dR2O+d;

                  double turn=Olx-Plx; //object error in robot world

                  while(abs(aORP)>0.2){


                      temp=ros::Time::now().toSec()-start;

                      update_robot_coordiantes(&robotino);
                      dR2O= sqrt((x-Ox)*(x-Ox)+(y-Oy)*(y-Oy)); //d(robot, object)
                      dR2P= sqrt((x-X[i])*(x-X[i])+(y-Y[i])*(y-Y[i]));  //d(robot, path point)
                      dO2P= sqrt(errX*errX+errY*errY);

                      aORP=acos((dR2P*dR2P-dO2P*dO2P+dR2O*dR2O)/(2*dR2O*dR2P));
                      if (aORP>3.14) aORP=aORP-2*3.14;
                      if (aORP<-3.14) aORP=aORP+2*3.14;

                      if (turn<0) robotino.singleMove( v_circ*sin(temp), v_circ*cos(temp), 0, 0, 0, 0);//turn left with respect to object
                      else robotino.singleMove( v_circ*sin(temp), -v_circ*cos(temp), 0, 0, 0, 0);//turn right  with respect to object
                     // cout<<"angle "<<aRPO<< " "<< dR2P<< " "<<dO2P<< endl;
                      //cout<<"turn "<<turn<<endl;
                      ros::spinOnce();
                      lRate.sleep();

                  }
                  update_robot_coordiantes(&robotino);
                  double lost=0;
                  lost=!update_object_coordiantes();
                  while(lost){
                      if (turn>0) robotino.singleMove( 0, 0, 0, 0, 0, 0.1);
                      else robotino.singleMove( 0, 0, 0, 0, 0, -0.1);
                      lost=!update_object_coordiantes();
                  }
                 // face_object(&robotino,aR2O);

                 // i=i-1;
                  }
           /* else if (sgn(errTh)*errTh> 0.5){
                cout<<"here 2"<<endl;
                robotino.singleMove( 0, 0, 0, 0, 0, omega);
                 oFileVR << "\t" << 0 << "\t" << 0 << "\t" <<omega<<endl;
              //  cout<<"here 1"<<endl;
            }*/
            else if (sgn(errY)*errY> 0.05){
                cout<<"here 3"<<endl;
               //if(vx>0.2)vx=0.2;
               //if(vy>0.2)vy=0.2;
                robotino.singleMove( vx, vy, 0, 0, 0, 0);
                 oFileVR << "\t" << vx << "\t" << vy << "\t" <<0<<endl;
               // cout<<"here 2"<<endl;
            }
           /* else if (errX<-0.05)
            {
                while(temp<6.4){
                    //  cout<< "here 2"<<endl;
                    ros::spinOnce();
                    //  cout<< "here 3"<<endl;
                    if(sgn(Olx)>0) robotino.singleMove(-1*Olx*cos(temp), -1*Olx*sin(temp), 0, 0, 0, 0.1); //udesno i unazad
                  else robotino.singleMove( -1*Olx*cos(temp), Olx*sin(temp), 0, 0, 0, -0.1);

                   // cout <<"cos "<<cos(temp)<<" sin "<<sin(temp)<<endl;
                    temp=temp+deltat;
                    lRate.sleep();

                }
            }*/
                else
            {
                cout<<"here 4"<<endl;
               //if(vx>0.2)vx=0.2;
              // if( vx>0){ //da ne ide unazad
             robotino.singleMove( vx, 0, 0, 0, 0, 0);
             oFileVR << "\t" <<vx << "\t" << 0 << "\t" <<0<<endl;
              // }
            }

           if (i<p-1)i++;
           else{
               cout<<"end"<<endl<<"current errors x: "<<errX<<" y:" <<errY<<endl;
           }
            lRate.sleep();

         }

         break;
          }
     case 14:
          {



         if( rO.call(R))
         {cout<<"call success"<<endl;}
         else{ cout<<"call not successful"<<endl; }
         sleep(5);

         cout<<"PUT THE OBJECT IN FRONT OF THE ROBOT"<<endl<<endl;

         //reading from file
         ifstream inFile;
         string filei="/home/c7031098/testing/robotino/"+tr;
         inFile.open (filei.c_str(),ios::in);
         string val;

         string pathR="/home/c7031098/testing/robotino/pathR"+expN;
         ofstream oFileR;
         oFileR.open(pathR.c_str());

         string errR="/home/c7031098/testing/robotino/errR"+expN;
         ofstream oFileER;
         oFileER.open(errR.c_str());

         string velR="/home/c7031098/testing/robotino/velR"+expN;
         ofstream oFileVR;
         oFileVR.open(velR.c_str());

         string pathO="/home/c7031098/testing/robotino/pathO"+expN;
         ofstream oFileO;
         oFileO.open(pathO.c_str());


         string pathOg="/home/c7031098/testing/robotino/pathOg"+expN;
         ofstream oFileOg;
         oFileOg.open(pathOg.c_str());

         cout<<"file open"<<endl;

         double n = 0;

         while(!inFile.eof()) {

             for(int i = 0; i < 3; ++i) {
                inFile>>val;
                 n = string_to_double(val);
                 if(i == 0) { X.push_back(n+0.05); }
                 else if(i == 1) { Y.push_back(n); }
                 else { TH.push_back(n); }
             }
             fflush(stdout);
         }



         update_robot_coordiantes(&robotino);
         update_object_coordiantes();
         //path tracking
         vector<double> Ex,Ey,Eth, dEx,dEy,dEth,vOlx,vOly,vOlth;
          double dR2O, dR2P, dO2P, aRPO, aR2O,aORP, aO2P;

         double vx,vy,omega,derrX,derrY,derrTh;
         int p=X.size();
         int i=0;
         double start =ros::Time::now().toSec();
         while ((i<p-1)||((i==p-1)&&((abs(X[p-1]-Ox)>0.1)||(abs(Y[p-1]-Oy)>0.1)))){ //error tolerance of 5 cm

             ros::spinOnce();
             //robotino position in /odom world
             update_robot_coordiantes(&robotino);
             update_object_coordiantes();


             //checking distance of robot to the path and  robot to the object
             //distance between robot and path has to be longer then robot to the object

              dR2O= sqrt((x-Ox)*(x-Ox)+(y-Oy)*(y-Oy)); //d(robot, object)
              dR2P= sqrt((x-X[i])*(x-X[i])+(y-Y[i])*(y-Y[i]));  //d(robot, path point)
              dO2P= sqrt(errX*errX+errY*errY);

            // cout<<"distances: r-o "<< dR2O<<" r-p "<< dR2P <<" o-p "<<dO2P<<endl;

             aORP=acos((dR2P*dR2P-dO2P*dO2P+dR2O*dR2O)/(2*dR2O*dR2P));
             if (aORP>3.14) aORP=aORP-2*3.14;
             if (aORP<-3.14) aORP=aORP+2*3.14;

             aR2O=atan2(Oy-y,Ox-x);
             if(aR2O>3.14)aR2O=aR2O-3.14;
             if(aR2O<-3.14)aR2O=aR2O+3.14;

             aO2P=atan2(Y[i]-Oy,X[i]-Ox);
             if(aO2P>3.14)aO2P=aO2P-3.14;
             if(aO2P<-3.14)aO2P=aO2P+3.14;

             //calculating local position of point from path
             double Plx=(X[i]-x)*sin(th)-(Y[i]-y)*cos(th);
             double Ply=-(X[i]-x)*cos(th)-(Y[i]-y)*sin(th);


            vOlx.push_back(Olx);
            vOly.push_back(Oly);
            vOlth.push_back(Olth);

           /* if(i>5){
                bool lost=1;
                for (int k=1;k<6;k++){

                   // cout<<"testing object lost i-k: "<<i-k<<" x(i-k): "<<vOlx[i-k]<<" x: "<<Olx<<" y(i-k): "<<vOly[i-k]<<" x: "<<Oly<<  endl;
                    if ((vOlx[i-k]!=Olx)||(vOly[i-k]!=Oly)||(vOlth[i-k]!=Olth)) lost=0;

                }
                object_lost=lost;
                if(lost){
                    robotino.singleMove(0, 0, 0, 0, 0, 0);
                    lRate.sleep();
                   ros::spinOnce();
                   cout <<"object lost"<<endl; //to replace with mesage for planner
                   //rotational movement of robot for object search- to implement
                   break;

                }
            }*/



            //calculating errors

            if (i==p-1){
                cout<<"recalculating plan"<<endl;
                int pom=p-i;
                double relx, rely;
                relx=100; rely=100;
                for(int j=0; j<p-1; j++)
                {
                   // cout<<"tren "<<sqrt(relx*relx+rely*rely)<<endl;
                    //cout<<"moguca "<<sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy))<<endl;
                        if (sqrt(relx*relx+rely*rely)>sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy)))
                        {
                            relx=X[j]-Ox;
                            rely=Y[j]-Oy;
                            pom=j;
                        }
                }
                i=pom;
            }


            errTh=TH[i]-Oth;
            errX=X[i]-Ox;
            errY=Y[i]-Oy;
           // {cout<<"current errors x: "<<errX<<" y:" <<errY<<" errTh: "<<errTh<<endl;}



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

            oFileR << "\t" << x << "\t" << y << "\t" <<yaw<<endl;
            oFileO << "\t" <<Olx << "\t" <<Oly << "\t" <<Olth<<endl;
            oFileOg << "\t" <<Ox << "\t" <<Oy << "\t" <<Oth<<endl;
            oFileER << "\t" << X[i] << "\t" << Y[i] << "\t" <<errTh<<endl;


           /* if((sgn(Olx)*Olx>0.04)&&((Plx-Olx)<0)){
                cout<<"here 1"<<endl;
                robotino.singleMove( -Oly, -1*Olx, 0, 0, 0, 0);
                i=i-1;
                 oFileVR << "\t" << 0 << "\t" << -1*Olx<< "\t" <<omega<<endl;
b
            }*/
            cout<<endl<<"object l "<<Olx<<"   "<<Oly<<endl;
            cout<<" plx "<<Plx<<" Ply "<< Ply<<endl;
            cout<<"robot "<<x<<"   "<<y<<"  "<<th<<"   "<<endl;
            cout<<"object "<<Ox<<"   "<<Oy<<endl;
            cout<<"wanted "<<X[i]<<"   "<<Y[i]<<endl;

          //  if (Y[i]<-0.05)break;
            // cout<<endl<<" aR2O "<<aR2O<<" robot th: "<<th<< " difference "<<th-aR2O<<endl;

       if ((((Olx>0)&&(Plx<Olx)||(Olx<0)&&(Plx>Olx))||(Ply>0.02))&&(aORP>0.3)||(vx<0)){

               //  cout<<"R2P "<<dR2P<<" O2P " <<dO2P<<" R2O " <<dR2O<<" ugao "<<aORP<<endl;
               cout<<"action 3"<<endl;

               cout<<"difference "<< Plx-Olx<<" aORP "<<aORP<<" vx "<<vx<<endl;
              // sleep(5);

              // cout<<" plx "<<Plx<<" olx "<< Olx<<" aORP "<<aORP<<endl;

               //  double v_circ=dR2O+d;

                // double temp=ros::Time::now().toSec()-start;

                 //    if (Olx<0) robotino.singleMove( v_circ*sin(temp), v_circ*cos(temp), 0, 0, 0, 0);//turn left with respect to object
                  //   else robotino.singleMove( v_circ*sin(temp), -v_circ*cos(temp), 0, 0, 0, 0);//turn right  with respect to object
                 //if (Olx-Plx<0) robotino.singleMove( 0, Plx-Olx, 0, 0, 0, 0);//turn left with respect to object
                // else robotino.singleMove( 0, -(Plx-Olx), 0, 0, 0, 0);//turn right  with respect to object
                 robotino.singleMove( 0, 1.5*(Plx-Olx), 0, 0, 0, 0);//turn left with respect to object
                 oFileVR << "\t" <<0 << "\t" << 1.5*(Plx-Olx) << "\t" <<0<<"\t"<<3<<"\t"<<i<<endl;


                  i=i-1;
                 }
            else if((abs(th-aO2P)>0.3)&&(dO2P>0.1)&&(abs(Olx)>0.03)&&(abs(th-aR2O)<1.2)&&(abs(Plx-Olx)>0.08)){
                 cout<<"action 0 "<<endl;
                 cout<<"aO2P "<<aO2P<<" robot th: "<<th<< " difference "<<th-aO2P<<" dO2P "<<dO2P<<" th-aR2O "<<th-aR2O<< endl;
                 //face_object(&robotino,aR2O);
                 double vth;
                 if ((aO2P-th)<3.14 && (sgn(aO2P)!=sgn(th)))vth=0.3*(6.28-aO2P-th);
                 else if ((aO2P-th)>3.14 && (sgn(aO2P)!=sgn(th)))vth=0.3*(aO2P-6.28-th);
                 else vth=0.3*(aO2P-th);
                 robotino.singleMove(0,0,0,0,0,0.3*(aO2P-th));
                 oFileVR << "\t" <<0 << "\t" << 0 << "\t" <<0.3*(aO2P-th)<<"\t"<<0<<"\t"<<i<<endl;

                 i=i-1;
            }
             //else if((abs(th-aR2O)>0.1)||(abs(th-aO2P))>0.2){
            else if ((abs(th-aR2O)>0.2)&&(dO2P<0.1)){
                 cout<<"action 1 "<<endl;
                 cout<<"aR2O"<<aR2O<<" robot th: "<<th<< "difference"<<th-aR2O<<endl;
                 //face_object(&robotino,aR2O);
                 robotino.singleMove(0,0,0,0,0,0.3*(aR2O-th));
                  oFileVR << "\t" <<0 << "\t" << 0 << "\t" <<0.3*(aR2O-th)<<"\t"<<1<<"\t"<<i<<endl;
                 i=i-1;
            }
           // else if ((abs(Olx)>0.15)||(abs(Olx)>0.10)&&((Olx>0)&&(Plx<Olx)||(Olx<0)&&(Plx>Olx))||(Oly<-(d+0.10))){


           // else if (((Plx-Olx)>0.02)||(aORP>0.3)||(vx<0)){

            else if ((abs(Olx)>0.10)||(abs(Olx)>0.05)&&((Olx>0)&&(Plx<Olx)||(Olx<0)&&(Plx>Olx))||(Oly<-(d+0.10))){
                 cout<<"action 2 "<<endl;
                 cout<<"x "<<0.5*(abs(Oly)-d)<<" y "<<-Olx<<endl;
                 robotino.singleMove( 0.3*(abs(Oly)-d), -1*Olx, 0, 0, 0,0);
                 oFileVR << "\t" <<0.3*(abs(Oly)-d) << "\t" << -1*Olx<< "\t" <<0<<"\t"<<2<<"\t"<<i<<endl;
                 i=i-1;

            }

            else if (sgn(errY)*errY> 0.05){
                cout<<"here 4"<<endl;
               //if(vx>0.2)vx=0.2;
               //if(vy>0.2)vy=0.2;
                cout<<"vx "<<vx<<" vy "<<vy<<endl;
                robotino.singleMove( vx, vy, 0, 0, 0, 0);
                 oFileVR << "\t" <<vx << "\t" << vy<< "\t" <<0<<"\t"<<4<<"\t"<<i<<endl;
               // cout<<"here 2"<<endl;
            }
           /* else if (errX<-0.05)
            {
                while(temp<6.4){
                    //  cout<< "here 2"<<endl;
                    ros::spinOnce();
                    //  cout<< "here 3"<<endl;
                    if(sgn(Olx)>0) robotino.singleMove(-1*Olx*cos(temp), -1*Olx*sin(temp), 0, 0, 0, 0.1); //udesno i unazad
                  else robotino.singleMove( -1*Olx*cos(temp), Olx*sin(temp), 0, 0, 0, -0.1);

                   // cout <<"cos "<<cos(temp)<<" sin "<<sin(temp)<<endl;
                    temp=temp+deltat;
                    lRate.sleep();

                }
            }*/
                else
            {
                cout<<"here 5"<<endl;
               //if(vx>0.2)vx=0.2;
              // if( vx>0){ //da ne ide unazad
             robotino.singleMove( vx, 0, 0, 0, 0, 0);
             oFileVR << "\t" <<vx << "\t" << 0 << "\t" <<0<<endl;
              // }
            }

           if (i<p-1)i++;
           else{
               cout<<"end"<<endl<<"current errors x: "<<errX<<" y:" <<errY<<endl;
           }
            lRate.sleep();

         }

         break;
          }


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

void update_robot_coordiantes(RobotinoControl *robotino){
   // cout <<"update robot"<<endl;
     nav_msgs::Odometry Odometry;
    //robotino position in /odom world
    Odometry = robotino->getOdom();
    x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;
    th=tf::getYaw(Odometry.pose.pose.orientation);


}

bool update_object_coordiantes(){
   // cout <<"update object"<<endl;
    ros::Rate lRate(5);
    int lost_count=0; //counting how many times object was lost for triggering search of the object
    //relative position of the object
    int check=0;
    while(!check){
        Olx=t.transform.translation.x -offsetX1;
        Oly=t.transform.translation.y -offsetY1;
        Olth=tf::getYaw(t.transform.rotation) -offsetTh1;

       /* if(!isnan(Olth))check=1;
        else {
             cout<<"reading nan"<<endl;
             ros::spinOnce();
             lRate.sleep();
             lost_count++;
        }*/
        if (lost_count==10){
            cout<<"Object lost"<<endl;
            return 0;
        }
        else
        {
            //global position of object in /odom
          //  Ox=x - Oly*cos(th)-Olx*sin(th);
           // Oy=y + Oly*sin(th)-Olx*cos(th);
            //Oth=th - Olth;

            Ox=x - Oly*cos(th)+Olx*sin(th);
            Oy=y - Oly*sin(th)-Olx*cos(th);
           Oth=th - Olth;




           // cout<<"here "<<Olx<<"   "<<Oly<<"  "<<Olth<<"   "<<endl;

            return 1;
        }
    }


}

void face_object(RobotinoControl *robotino, double aR2O){
    //turning robot to face the object
    double errTH=th-aR2O;
    //cout<<th<<"  "<<aR2O<<endl;
    //cout<<"error of robot orientation "<< errTH<<endl;
    ros::Rate lRate(5);

    while(abs(errTH)>0.05){

        update_robot_coordiantes(robotino);
        update_object_coordiantes();

        aR2O=atan2(Oy-y,Ox-x);  //angle robot to object -> inverse formula because of robot coordinate system
        if(aR2O>3.14)aR2O=aR2O-2*3.14;
        if(aR2O<-3.14)aR2O=aR2O+2*3.14;
      //  cout<<"current agle of the srobot"<< errTH<<endl;
         errTH=th-aR2O;
        if(errTH>0.02)robotino->singleMove( 0, 0, 0, 0, 0, -0.1);
        if(errTH<-0.02)robotino->singleMove( 0, 0, 0, 0, 0, 0.1);

        // cout<<th<<"  "<<aR2O<<endl;
       // cout<<"error of robot orientation "<< errTH<<endl;
        ros::spinOnce();
        lRate.sleep();

    }
}
