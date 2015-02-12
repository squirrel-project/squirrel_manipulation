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


string expN="s2d1102";

string tr="trajL2.txt";
bool firstSet = false;
double Rx0, Ry0, Rz0;
double Ox0, Oy0, Oz0;
double offsetTh, offsetX,offsetY; // ar tags calibration. inital offset

double temp=0;
double deltat=0.1;

void arCallback(tf::tfMessage msg);

template <typename T> int sgn(T val);
double string_to_double(const std::string& s);

vector<double> Ex,Ey,Eth, dEx,dEy,dEth, Eox,Eoy,EoTh;

double vx,vy,omega,derrX,derrY,derrTh, Ox,Oy,Oth,Olx,Oly,Olth,Ox1,Oy1,Oth1;

geometry_msgs::TransformStamped t;

int main(int argc, char** args) {

    int execute;
   // execute=0; //collect trajectory in odometry
   // execute=1; //modified pid with fixed parameters. path tracking for robot in odometry
    //execute=2; //modified pid with fixed parameters. path tracking for objectu in odometry
   //execute=3; //testing object tracking
   // execute=4; //elipse movement
    execute=5;// modified pid with fixed parameters. path tracking for robot in odometry + elipse movement //hardcoded some parameters
    //execute=6;// modified pid with fixed parameters. path tracking for object in odometry + elipse movement //hardcoded some parameters
   // execute=7;// modified pid with fixed parameters. path tracking for object in odometry + elipse movement //hardcoded some parameters
              //added reaching the goal
    //  execute=6; //MPC

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

    offsetX=t.transform.translation.x;
    offsetY=t.transform.translation.y;
    offsetTh=tf::getYaw(t.transform.rotation);
    cout<<"Offest for x is: "<<offsetX<<endl;
    cout<<"Offset for y is: "<<offsetY<<endl;
    cout<<"Offset for th is: "<<offsetTh<<endl<<endl;


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




    double x=initX; double  y=initY;  double th=tf::getYaw(Odometry.pose.pose.orientation);
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
         vector<double> Ex,Ey,Eth, dEx,dEy,dEth;

         double vx,vy,omega,derrX,derrY,derrTh;
         int p=X.size();
         int i=0;
         while ((i<p)||((i==p-1)&&(abs(X[p-1]-Ox)>0.05)&&(abs(Y[i]-Oy)>0.05))){ //error tolerance of 5 cm

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


            //calculating errors

            if (i==p-i){
                int pom=0;
                double relx, rely;
                relx=100; rely=100;
                for(int j=0; j<p-1; j++)
                {
                    cout<<"tren "<<sqrt(relx*relx+rely*rely)<<endl;
                    cout<<"moguca "<<sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy))<<endl;
                        if (sqrt(relx*relx+rely*rely)>sqrt((X[j]-Ox)*(X[j]-Ox)+(Y[j]-Oy)*(Y[j]-Oy)))
                        {
                            relx=X[j]-Ox;
                            rely=Y[j]-Oy;
                            pom=j;
                        }

                i=pom;
            }
            }


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

           }
           else*/
                if(sgn(Olx)*Olx>0.02){
                cout<<"here 1"<<endl;
                robotino.singleMove( 0, -1*Olx, 0, 0, 0, omega);
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
            else
            {
                cout<<"here 4"<<endl;
               //if(vx>0.2)vx=0.2;
              // if( vx>0){ //da ne ide unazad
             robotino.singleMove( vx, 0, 0, 0, 0, 0);
             oFileVR << "\t" <<vx << "\t" << 0 << "\t" <<0<<endl;
              // }
            }

           if (i<p)i++;
           else
           {cout<<"end"<<endl<<"current errors x: "<<errX<<" y:" <<errY<<endl;}
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


