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
geometry_msgs::PoseStamped Map2Base_link(double x, double y);
geometry_msgs::PoseStamped Base_link2Map(double x, double y);

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

     tf::TransformListener tf_listener;

    cout << "v4r everything initialized" << endl;

    while(!firstSet) {
        ros::spinOnce();
        lRate.sleep();
    }

    ros::spinOnce();

    nav_msgs::Odometry Odometry = robotino.getOdom();
    double initX = Odometry.pose.pose.position.x ; double initY = Odometry.pose.pose.position.y ;

    geometry_msgs::Quaternion rotation = t.transform.rotation;

    squirrel_rgbd_mapping_msgs::GetPushingPlan srvPlan;


   // cout<<"start pos x:"<<x<<" y: "<<y<<" theta: "<<th<<endl<<endl<<endl;

    //Getting pushing plan

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
        BOOST_ASSERT_MSG( srvPlan.response.plan.header.frame_id == "/map" ||
                          srvPlan.response.plan.header.frame_id == "map" ,
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

    int i=1;

    X.push_back(pushing_path.poses[0].pose.position.x);
    Y.push_back(pushing_path.poses[0].pose.position.y);
    TH.push_back(tf::getYaw(pushing_path.poses[0].pose.orientation));

   //getting coordinates separately

    while(i<path_length) {
       // X.push_back((pushing_path.poses[i].pose.position.x+pushing_path.poses[i-1].pose.position.x)/2);
       // Y.push_back((pushing_path.poses[i].pose.position.y+pushing_path.poses[i-1].pose.position.y)/2);
       // TH.push_back(tf::getYaw(pushing_path.poses[i].pose.orientation));

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

    double vx,vy,omega,derrX,derrY,derrTh,x,y,th;
    int p=X.size();

    i=0;
    while((i<p-1)||((abs(X[p-1]-x)>0.1)||(abs(Y[p-1]-y)>0.1))){


        ros::spinOnce();

      //  Odometry = robotino.getOdom();
       // x = Odometry.pose.pose.position.x  ;  y = Odometry.pose.pose.position.y ;
          x=pose_m_.x;
          y=pose_m_.y;

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

       cout<<"wanted x"<<X[i]<<"current x: "<<x <<"wanted y"<<Y[i]<<"current y: "<<y<< endl;


     //  errX=X[i]-x;
      // errY=Y[i]-y;


        //ROS_INFO("errX: %lf errY: %lf", errX, errY);
       //yaw=tf::getYaw(Odometry.pose.pose.orientation);
      // errTh=TH[i]-yaw;


        geometry_msgs::PoseStamped Emap, Eloc;

        Emap.pose.position.x=X[i];
        Emap.pose.position.y=Y[i];
          Emap.pose.position.z=0;
          Emap.pose.orientation.x=0;
          Emap.pose.orientation.y=0;
          Emap.pose.orientation.z=0;
          Emap.pose.orientation.w=1;
          Emap.header.frame_id="/map";
       try {
           tf_listener.waitForTransform("/map","/base_link", ros::Time::now(), ros::Duration(1.0));
           tf_listener.transformPose("/base_link",Emap,Eloc);
       } catch (tf::TransformException& ex) {
         std::string ns = ros::this_node::getNamespace();
         std::string node_name = ros::this_node::getName();
         ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

         return;
       }


       //SBth=yaw;


       rotation = t.transform.rotation;


       //Ex.push_back(errX);
       //Ey.push_back(errY);
       //Eth.push_back(errTh);

       errX=Eloc.pose.position.x;
       errY=Eloc.pose.position.y;

       Ex.push_back(Eloc.pose.position.x);
       Ey.push_back(Eloc.pose.position.y);
       if(i==1){
           derrX=0; derrY=0; derrTh=0;
       }
       else{
           derrX=errX-Ex[i-1];//derivatives
           derrY=errY-Ey[i-1];
          // derrTh=errTh-Eth[i-1];
       }


       dEx.push_back(derrX);
       dEy.push_back(derrY);
       dEth.push_back(derrTh);


       geometry_msgs::PoseStamped dEmap, dEloc;

         dEmap.pose.position.x=derrX;
         dEmap.pose.position.y=derrY;
         dEmap.pose.position.z=0;
         dEmap.pose.orientation.x=0;
         dEmap.pose.orientation.y=0;
         dEmap.pose.orientation.z=0;
         dEmap.pose.orientation.w=1;
         dEmap.header.frame_id="/map";
      try {
          tf_listener.waitForTransform("/map","/base_link", ros::Time::now(), ros::Duration(1.0));
          tf_listener.transformPose("/base_link",dEmap,dEloc);
      } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

        return;
      }
       cout<<"error before transformation x: "<<errX<<" y:"<<errY<<endl;

       cout<<"local error after transformation x: "<<Eloc.pose.position.x<<" y:"<<Eloc.pose.position.y<<endl<<endl;

      // vx=(0.4*(errX*cos(th)+errY*sin(th))+0.1*(derrX*cos(th)+derrY*sin(th)));
       //vy=0.2*(errY*cos(th)-errX*sin(th))+0.1*(derrY*cos(th)-derrX*sin(th));
       //omega=0.5*errTh;
         vx=0.4*Eloc.pose.position.x; //+0.1*dEloc.pose.position.x;
         vy=0.4*Eloc.pose.position.y;//+0.1*dEloc.pose.position.y;
       //Tho=tf::getYaw(rotation);

       //  ROS_INFO("vx: %lf vy: %lf", vx, vy);

         /* if(sgn(t.transform.translation.x)*t.transform.translation.x>0.02){
           robotino.singleMove( 0, -1*t.transform.translation.x, 0, 0, 0, 0);
           i=i-1;

       }
      // else*/
       /*    if (sgn(errTh)*errTh> 0.05){
           robotino.singleMove( 0, 0, 0, 0, 0, omega);


       }*/
      //else
      /* if (sgn(errY)*errY> 0.05){
           robotino.singleMove( vx, vy, 0, 0, 0, 0);

       }
       else
       {
        robotino.singleMove( vx, 0, 0, 0, 0, 0);

       }*/
       robotino.singleMove( vx, vy, 0, 0, 0, 0);
       i++;
       lRate.sleep();

     }
        double d=0.23;
        double offsetX=t.transform.translation.y+d;
        double offsetY=t.transform.translation.x;
        double Rx,Ry,Rth,Ox,Oy,Olx,Oly,Plx,Ply;
        double Vx,Vy,Vth;
        double dR2O,dR2P,dO2P,aORP,aR2O,aO2P,errXl,errYl, dErrXl,dErrYl;
        vector<double> Elx,Ely;

        geometry_msgs::PoseStamped transH;

    i=0;
    while ((i<p-1)||((i==p-1)&&((abs(X[p-1]-Ox)>0.1)||(abs(Y[p-1]-Oy)>0.1)))){ //error tolerance of 5 cm

                 ros::spinOnce();

                 //check if end of control sequence reached

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

                //checking object lost

                //update of coordinates

                 // /map robot
                 Rx=pose_m_.x;
                 Ry=pose_m_.y;
                 Rth=pose_m_.theta;

                 // /base_link object
                 Olx=-t.transform.translation.y+offsetX;
                 Oly=-t.transform.translation.x+offsetY;

                 // /map object
                 transH=Base_link2Map(Olx,Oly);
                 Ox=transH.pose.position.x;
                 Oy=transH.pose.position.y;

                 // /map object - X[i] and Y[i]

                 // /base_link object wanted
                 transH=Map2Base_link(X[i],Y[i]);
                 Plx=transH.pose.position.x;
                 Ply=transH.pose.position.y;


                 //calculating errors and derivatives

                 errXl=Plx-Olx;
                 errYl=Ply-Oly;

                 Elx.push_back(errXl);
                 Ely.push_back(errYl);

                 if(i==1){
                    dErrXl=0; dErrYl=0;}
                 else{
                    dErrXl=errXl-Elx[i-1];//derivatives
                    dErrYl=errYl-Ely[i-1];
                 }

                  Vx=0; Vy=0; Vth=0;

                 //checking distance of robot to the path and  robot to the object
                 //distance between robot and path has to be longer then robot to the object

                  dR2O= sqrt((Rx-Ox)*(Rx-Ox)+(Ry-Oy)*(Ry-Oy)); //d(robot, object)
                  dR2P= sqrt((Rx-X[i])*(Rx-X[i])+(Ry-Y[i])*(Ry-Y[i]));  //d(robot, path point)
                  dO2P= sqrt((Ox-X[i])*(Ox-X[i])+(Oy-Y[i])*(Oy-Y[i])); //d(object, path point) -> error

                // cout<<"distances: r-o "<< dR2O<<" r-p "<< dR2P <<" o-p "<<dO2P<<endl;

                 //checking angles of robot to the path and  robot to the object

                 aORP=acos((dR2P*dR2P-dO2P*dO2P+dR2O*dR2O)/(2*dR2O*dR2P));
                 if (aORP>3.14) aORP=aORP-2*3.14;
                 if (aORP<-3.14) aORP=aORP+2*3.14;

                 aR2O=atan2(Oy-y,Ox-x);
                 if(aR2O>3.14)aR2O=aR2O-3.14;
                 if(aR2O<-3.14)aR2O=aR2O+3.14;

                 aO2P=atan2(Y[i]-Oy,X[i]-Ox);
                 if(aO2P>3.14)aO2P=aO2P-3.14;
                 if(aO2P<-3.14)aO2P=aO2P+3.14;


                Vx=0.4*errXl+0.1*dErrXl;
                Vy=0.2*errYl+0.1*dErrYl;


                cout<<endl<<"object l "<<Olx<<"   "<<Oly<<endl;
                cout<<" plx "<<Plx<<" Ply "<< Ply<<endl;
                cout<<"robot "<<Rx<<"   "<<Ry<<"  "<<Rth<<"   "<<endl;
                cout<<"object "<<Ox<<"   "<<Oy<<endl;
                cout<<"wanted "<<X[i]<<"   "<<Y[i]<<endl;


           if ((((Oly<0)&&(Plx>Olx)||(Oly>0)&&(Plx<Olx))||(Plx<-0.02))&&(aORP>0.3)||(Vx<0)){

                   cout<<"action 3"<<endl;
                   cout<<"difference "<< Plx-Olx<<" aORP "<<aORP<<" vx "<<vx<<endl;

                   Vx=0; Vy=1.5*(Oly-Ply); Vth=0;
                  // robotino.singleMove( 0, 1.5*(Plx-Olx), 0, 0, 0, 0);//turn left with respect to object
                      i=i-1;
                     }
               else if((abs(th-aO2P)>0.3)&&(dO2P>0.1)&&(abs(Oly)>0.03)&&(abs(th-aR2O)<1.2)&&(abs(Ply-Oly)>0.08)){
                     cout<<"action 0 "<<endl;
                     cout<<"aO2P "<<aO2P<<" robot th: "<<th<< " difference "<<th-aO2P<<" dO2P "<<dO2P<<" th-aR2O "<<th-aR2O<< endl;

                     Vx=0; Vy=0;
                     if ((aO2P-th)<3.14 && (sgn(aO2P)!=sgn(th)))Vth=0.3*(6.28-aO2P-th);
                     else if ((aO2P-th)>3.14 && (sgn(aO2P)!=sgn(th)))Vth=0.3*(aO2P-6.28-th);
                     else Vth=0.3*(aO2P-th);

                     i=i-1;
                }
                 //else if((abs(th-aR2O)>0.1)||(abs(th-aO2P))>0.2){
               else if ((abs(th-aR2O)>0.2)&&(dO2P<0.1)){
                     cout<<"action 1 "<<endl;
                     cout<<"aR2O"<<aR2O<<" robot th: "<<th<< "difference"<<th-aR2O<<endl;

                     Vx=0; Vy=0; Vth=0.3*(aR2O-th);

                     i=i-1;
                }
               // else if ((abs(Olx)>0.15)||(abs(Olx)>0.10)&&((Olx>0)&&(Plx<Olx)||(Olx<0)&&(Plx>Olx))||(Oly<-(d+0.10))){

               // else if (((Plx-Olx)>0.02)||(aORP>0.3)||(vx<0)){

                else if ((abs(Oly)>0.10)||(abs(Oly)>0.05)&&((Oly<0)&&(Ply>Oly)||(Oly>0)&&(Ply<Oly))||(Olx>(d+0.10))){
                     cout<<"action 2 "<<endl;
                     cout<<"x "<<0.3*(abs(Olx)-d)<<" y "<<Oly<<endl;

                     Vx=0.3*(abs(Olx)-d); Vy=Oly; Vth=0;

                     //robotino.singleMove( 0.3*(abs(Oly)-d), -1*Olx, 0, 0, 0,0);

                     i=i-1;

                }

                else if (sgn(errYl)*errYl> 0.05){
                    cout<<"here 4"<<endl;
                   //if(vx>0.2)vx=0.2;
                   //if(vy>0.2)vy=0.2;
                    Vx=0.4*errXl+0.1*dErrXl;
                    Vy=0.2*errYl+0.1*dErrYl;
                    Vth=0;

                    cout<<"Vx "<<Vx<<" Vy "<<Vy<<endl;

                }

                    else
                {
                    cout<<"here 5"<<endl;
                   //if(vx>0.2)vx=0.2;
                  // if( vx>0){ //da ne ide unazad
                    Vx=0.4*errXl+0.1*dErrXl;
                    Vy=0;
                    Vth=0;
                    cout<<"Vx "<<Vx<<" Vy "<<Vy<<endl;

                  // }
                }

               if (i<p-1)i++;
               else{
                   cout<<"end"<<endl<<"current errors x: "<<errX<<" y:" <<errY<<endl;
               }

               robotino.singleMove(Vx,Vy,0.0,0.0,0.0,Vth);
                lRate.sleep();

             }


    /* for(int i = 1; i <= 10; ++i) {

        rate.sleep();
        PushFeedback.percent_completed = i * 10;
        pushServer.publishFeedback(PushFeedback);

    }*/

    pushResult.result_status = "success";
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

geometry_msgs::PoseStamped Map2Base_link(double x, double y){

    geometry_msgs::PoseStamped Emap, Eloc;
     tf::TransformListener tf_listener;

    Emap.pose.position.x=x;
    Emap.pose.position.y=y;
    Emap.pose.position.z=0;
    Emap.pose.orientation.x=0;
    Emap.pose.orientation.y=0;
    Emap.pose.orientation.z=0;
    Emap.pose.orientation.w=1;
    Emap.header.frame_id="/map";
    try {
        tf_listener.waitForTransform("/map","/base_link", ros::Time::now(), ros::Duration(1.0));
        tf_listener.transformPose("/base_link",Emap,Eloc);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

    }

    return Eloc;
}

geometry_msgs::PoseStamped Base_link2Map(double x, double y){

    geometry_msgs::PoseStamped Emap, Eloc;
     tf::TransformListener tf_listener;

    Emap.pose.position.x=x;
    Emap.pose.position.y=y;
    Emap.pose.position.z=0;
    Emap.pose.orientation.x=0;
    Emap.pose.orientation.y=0;
    Emap.pose.orientation.z=0;
    Emap.pose.orientation.w=1;
    Emap.header.frame_id="/map";
    try {
        tf_listener.waitForTransform("/base_link","/map", ros::Time::now(), ros::Duration(1.0));
        tf_listener.transformPose("/map",Emap,Eloc);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

    }

    return Eloc;
}
