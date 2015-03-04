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

#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <squirrel_object_manipulation/pushing.hpp>
#include <boost/assert.hpp>
#include<tf/transform_listener.h>

using namespace std;
using namespace ros;


string expN="trajOlog3";

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

    execute=1;



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

    update_robot_coordiantes(&robotino);
    //double x=initX; double  y=initY;  double th=tf::getYaw(Odometry.pose.pose.orientation);
    double  initth=th;
    double  yaw=th;

    cout<<"start pos x:"<<x<<" y: "<<y<<" z: "<<initZ<<" theta: "<<th<<endl<<endl<<endl;

     double errX,errY,errTh, eOX,eOY,eOTh;
     vector<double> X,Y,TH,Xo,Yo,THo;

     double Xo1,Yo1,Tho1, Tho;

     switch (execute){

     case 1:
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
     case 2:
          {



         if( rO.call(R))
         {cout<<"call success"<<endl;}
         else{ cout<<"call not successful"<<endl; }
         sleep(5);

         nav_msgs::Odometry Odometry = robotino.getOdom();
         double initX = Odometry.pose.pose.position.x ; double initY = Odometry.pose.pose.position.y ;

         squirrel_rgbd_mapping_msgs::GetPushingPlan srvPlan;


          x=initX;
          y=initY;
          th=tf::getYaw(Odometry.pose.pose.orientation);
          yaw=th;

         geometry_msgs::PoseStamped np, op;

         op.pose.position=Odometry.pose.pose.position;
         op.pose.orientation=Odometry.pose.pose.orientation;
         op.header.frame_id="/odom";
         tf::TransformListener tf2;
         tf2.transformPose("/map",op, np);


         cout<<"start pos x:"<<x<<" y: "<<y<<" theta: "<<th<<endl<<endl<<endl;

         srvPlan.request.start.x=np.pose.position.x;
         srvPlan.request.start.y=np.pose.position.y;
         srvPlan.request.start.theta=tf::getYaw(np.pose.orientation);
         Pose goal;

         srvPlan.request.goal.x=goal->pose.position.x;
         srvPlan.request.goal.y=goal->pose.position.y;
         srvPlan.request.goal.theta=tf::getYaw(goal->pose.orientation);


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


           nav_msgs::Path pushing_path=srvPlan.response.plan;

           int p= pushing_path.poses.size();

           //getting coordinates separately

            while(i<p) {

                X.push_back(pushing_path.poses[i].pose.position.x);
                Y.push_back(pushing_path.poses[i].pose.position.y);
                TH.push_back(tf::getYaw(pushing_path.poses[i].pose.orientation));
                i++;
            }


         cout<<"PUT THE OBJECT IN FRONT OF THE ROBOT"<<endl<<endl;

         update_robot_coordiantes(&robotino);
         update_object_coordiantes();
         //path tracking
         vector<double> Ex,Ey,Eth, dEx,dEy,dEth,vOlx,vOly,vOlth;
         double dR2O, dR2P, dO2P, aRPO, aR2O,aORP, aO2P;

         double vx,vy,omega,derrX,derrY,derrTh;
         int i=0;

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



            //calculating errors

            if (i==p-1){
                cout<<"recalculating plan"<<endl;
                int pom=p-i;
                double relx, rely;
                relx=100; rely=100;
                for(int j=0; j<p-1; j++)
                {
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

            cout<<endl<<"object l "<<Olx<<"   "<<Oly<<endl;
            cout<<" plx "<<Plx<<" Ply "<< Ply<<endl;
            cout<<"robot "<<x<<"   "<<y<<"  "<<th<<"   "<<endl;
            cout<<"object "<<Ox<<"   "<<Oy<<endl;
            cout<<"wanted "<<X[i]<<"   "<<Y[i]<<endl;


       if ((((Olx>0)&&(Plx<Olx)||(Olx<0)&&(Plx>Olx))||(Ply>0.02))&&(aORP>0.3)||(vx<0)){

               //  cout<<"R2P "<<dR2P<<" O2P " <<dO2P<<" R2O " <<dR2O<<" ugao "<<aORP<<endl;
               cout<<"action 3"<<endl;

               cout<<"difference "<< Plx-Olx<<" aORP "<<aORP<<" vx "<<vx<<endl;
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

                else
            {

            robotino.singleMove( vx, 0, 0, 0, 0, 0);
             oFileVR << "\t" <<vx << "\t" << 0 << "\t" <<0<<endl;

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
