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
#include <std_msgs/Float64.h>
#include <squirrel_object_manipulation/pushing.hpp>
#include <boost/assert.hpp>
#include <boost/range/numeric.hpp>
#include "mongodb_store/message_store.h"
// HACK Michael
#include <squirrel_object_perception_msgs/StartObjectTracking.h>
#include <squirrel_object_perception_msgs/StopObjectTracking.h>
// HACK END Michael

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



using namespace std;

bool firstSet = false;
double Rx0, Ry0, Rz0;
double Ox0, Oy0, Oz0;

void arCallback(tf::tfMessage msg);
geometry_msgs::PoseStamped Map2Base_link(double x, double y);
geometry_msgs::PoseStamped Base_link2Map(double x, double y);
geometry_msgs::PoseStamped odom2Map(double x, double y);
geometry_msgs::PoseStamped Kinect2Base_link(double x, double y, double z);
bool startTracking(std::string object_id);
bool stopTracking();

geometry_msgs::PoseStamped Odom2Base_link(double x, double y);
geometry_msgs::PoseStamped Base_link2Odom(double x, double y);

template <typename T> int sgn(T val);
double string_to_double(const std::string& s);

geometry_msgs::TransformStamped t;
tf::StampedTransform trans;

PushAction::PushAction(const std::string std_PushServerActionName) :
    pushServer(nh, std_PushServerActionName, boost::bind(&PushAction::executePush, this, _1), false),
    private_nh("~")
{
    pushServer.start();
    private_nh.param<std::string>("pose_topic", pose_topic_,"/squirrel_localizer_pose");
    TiltPub=nh.advertise<std_msgs::Float64>("/tilt_controller/command", 1);
    ImgPub = nh.advertise<sensor_msgs::Image>("/static_image", 1);

    pose_sub_ = nh.subscribe(pose_topic_, 2, &PushAction::updatePose, this);
    robotino = new RobotinoControl(nh);
}

PushAction::~PushAction() {
    cout << "************* before delete robotino\n";
    delete robotino;
    cout << "************* after delete robotino\n";
}

void PushAction::executePush(const squirrel_manipulation_msgs::PushGoalConstPtr &goal) {


    // first move the camera so that the object is in view
    //ros::Publisher TiltPub=nh.advertise<std_msgs::Float64>("/tilt_controller/command", 1);
    std_msgs::Float64 tilt_msg;
    tilt_msg.data = 0.66;
    TiltPub.publish(tilt_msg);
    ros::spinOnce();

    // publish info to the console for the user
    ROS_INFO("Push: (push up action) started push up of %s for manipulation",  goal->object_id.c_str());

    // HACK Michael
    if(startTracking(goal->object_id))
    {
      ROS_INFO("Push: (push up action) started tracking of %s",  goal->object_id.c_str());
    }
    else
    {
      ROS_ERROR("Push: (push up action) starting tracking of %s failed",  goal->object_id.c_str());
      pushResult.result_status = "failure";
      pushServer.setSucceeded(pushResult);
      return;
    }

    tf::TransformListener tf_listener;

    //ros::Subscriber markerSub = nh.subscribe("arMarker/tf", 1, arCallback);
    ros::Rate lRate(10);

    try {
      tf_listener.waitForTransform("base_link", goal->object_id, ros::Time::now(), ros::Duration(10.0));
      tf_listener.lookupTransform("base_link", goal->object_id, ros::Time(0), trans);
    } catch (tf::TransformException& ex) {
      // NOTE: if I don't get the first object pose, I give up
      std::string ns = ros::this_node::getNamespace();
      std::string node_name = ros::this_node::getName();
      ROS_ERROR("Push: %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
      pushResult.result_status = "failure";
      pushServer.setAborted(pushResult);
      stopTracking();
      return;
    }

   // t.transform.translation.x = trans.getOrigin().x();
    //t.transform.translation.y = trans.getOrigin().y();
    // HACK END Michael

    double d=0.23;
  //  double offsetX=t.transform.translation.y+d;
   // double offsetY=t.transform.translation.x;
    double Rx,Ry,Rth,Ox,Oy,Olx,Oly,Plx,Ply;
    double Vx,Vy,Vth;
    double dR2O,dR2P,dO2P,aORP,aR2O,aO2P,errXl,errYl, dErrXl,dErrYl;
   // vector<double>vOlx,vOly;
    vector<double> vOlx,vOly,Elx,Ely, vPlx, vPly, va1, va2,va3,va4x,va4y,  vElx,  vEly;


    geometry_msgs::PoseStamped transH;



/*S_INFO("Push: visualisation starting");
    //visualisation

cout<<"visualisation started"<<endl;
    cv_bridge::CvImage cv_image, cv_image1, cv_image2, cv_image3,cv_image4, cv_image5, cv_image6, cv_image7,cv_image8;
    cv_image.image = cv::imread("/home/robotino/git/robotino/src/squirrel_manipulation/squirrel_object_manipulation/include/diagram1.png",CV_LOAD_IMAGE_COLOR);
    cv_image1.image = cv::imread("/home/robotino/git/robotino/src/squirrel_manipulation/squirrel_object_manipulation/include/c1.png",CV_LOAD_IMAGE_COLOR);
    cv_image2.image = cv::imread("/home/robotino/git/robotino/src/squirrel_manipulation/squirrel_object_manipulation/include/c2.png",CV_LOAD_IMAGE_COLOR);
    cv_image3.image = cv::imread("/home/robotino/git/robotino/src/squirrel_manipulation/squirrel_object_manipulation/include/c3.png",CV_LOAD_IMAGE_COLOR);
    cv_image4.image = cv::imread("/home/robotino/git/robotino/src/squirrel_manipulation/squirrel_object_manipulation/include/c4.png",CV_LOAD_IMAGE_COLOR);
    cv_image5.image = cv::imread("/home/robotino/git/robotino/src/squirrel_manipulation/squirrel_object_manipulation/include/main.png",CV_LOAD_IMAGE_COLOR);
    cv_image6.image = cv::imread("/home/robotino/git/robotino/src/squirrel_manipulation/squirrel_object_manipulation/include/success.png",CV_LOAD_IMAGE_COLOR);
    cv_image7.image = cv::imread("/home/robotino/git/robotino/src/squirrel_manipulation/squirrel_object_manipulation/include/object.png",CV_LOAD_IMAGE_COLOR);
    cv_image8.image = cv::imread("/home/robotino/git/robotino/src/squirrel_manipulation/squirrel_object_manipulation/include/obstacle.png",CV_LOAD_IMAGE_COLOR);

    cv_image1.encoding = "bgr8";
    cv_image5.encoding = "bgr8";
    cv_image2.encoding = "bgr8";
    cv_image3.encoding = "bgr8";
    cv_image4.encoding = "bgr8";
    cv_image5.encoding = "bgr8";
    cv_image6.encoding = "bgr8";
    cv_image7.encoding = "bgr8";
    cv_image8.encoding = "bgr8";

    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);



     ImgPub.publish(ros_image);
     ros::spinOnce();
     lRate.sleep();
     cout<<"Image published"<<endl;
      int pom=0;
     while (pom<10){
          ImgPub.publish(ros_image);
          lRate.sleep();
          pom++;
          }
  ROS_INFO("Push: visualisation started");
 /* mongodb_store::MessageStoreProxy database(nh);

    vector< boost::shared_ptr < geometry_msgs::PoseStamped> > dataresults;


  // if(database.queryNamed< geometry_msgs::PoseStamped >(goal->object_id.c_str(),dataresults)){
    if(database.queryNamed< geometry_msgs::PoseStamped >("wp0",dataresults)){
       if(dataresults.size()<1){
           ROS_INFO("nothing recived from database");
           return;
       }
   }
   else {
       ROS_ERROR("database call failed");
       return;
   }


   geometry_msgs::PoseStamped &object=*dataresults[0];

   cout<<" x position of recivedpoint"<<object.pose.position.x<<endl;*/




    /*while(!firstSet) {
        ros::spinOnce();
        lRate.sleep();
    }*/
   // cout << "v4r everything initialized" << endl;

    ros::spinOnce();

    squirrel_rgbd_mapping_msgs::GetPushingPlan srvPlan;


   // cout<<"start pos x:"<<x<<" y: "<<y<<" theta: "<<th<<endl<<endl<<endl;

     ROS_INFO("Push: getting object position");

    //checking object position

    // HACK Michael
   // t.transform.translation.x = trans.getOrigin().x();
   // t.transform.translation.y = trans.getOrigin().y();
    // HACK END Michael



    // /base_link object
   //Olx=-t.transform.translation.y+offsetX;
    //Oly=-t.transform.translation.x+offsetY;

    /*cout<<"Object position in kinect_rgb_optical_frame x: "<<trans.getOrigin().x()<<" y "<<trans.getOrigin().y()<<endl;
    transH=Kinect2Base_link(trans.getOrigin().x(),trans.getOrigin().y(),trans.getOrigin().z());
    Olx=transH.pose.position.x;
    Oly=transH.pose.position.y;*/
    Olx=trans.getOrigin().x();
    Oly=trans.getOrigin().y();
    cout<<"Object position in base_link x: "<<Olx<<" y "<<Oly<<endl;


    // /map object
    transH=Base_link2Map(Olx,Oly);
    Ox=transH.pose.position.x;
    Oy=transH.pose.position.y;

    cout<<"Object position in map x: "<<Ox<<" y "<<Oy<<endl;

    //Getting pushing plan

    ROS_INFO("Push: requesting plan");
    cout<<endl;
    srvPlan.request.start.x = Ox;
    srvPlan.request.start.y = Oy;
    srvPlan.request.start.theta = pose_m_.theta;

    cout << "received object target position: " << goal->pose.position.x << " "  << goal->pose.position.y << "\n";
    srvPlan.request.goal.x = goal->pose.position.x;
    srvPlan.request.goal.y = goal->pose.position.y;
    srvPlan.request.goal.theta = tf::getYaw(goal->pose.orientation);

    geometry_msgs::Point32 p1, p2, p3, p4;

    p1.x = 0.25; p1.y = -0.25;
    p2.x = 0.25; p1.y = 0.25;
    p3.x = -0.20; p1.y = -0.20;
    p4.x = -0.20; p1.y = 0.20;

    srvPlan.request.object.points.push_back(p1);
    srvPlan.request.object.points.push_back(p2);
    srvPlan.request.object.points.push_back(p3);
    srvPlan.request.object.points.push_back(p4);

    if ( ros::service::call("/getPushingPlan", srvPlan) ) {
      if ( srvPlan.response.plan.poses.empty() ) {
        ROS_WARN("Push:got an empty plan");
       pushResult.result_status = "failure";
      pushServer.setAborted(pushResult);
      stopTracking();
      return;

      } else {
        BOOST_ASSERT_MSG( srvPlan.response.plan.header.frame_id == "/odom" ||
                          srvPlan.response.plan.header.frame_id == "odom" ,
                          "returned path is not in '/odom' frame");
        ROS_INFO("Push: got a path for pushing");
      }
    } else {
      ROS_ERROR("Push: unable to communicate with /getPushingPlan");
      pushResult.result_status = "failure";
      pushServer.setAborted(pushResult);
      stopTracking();
      return;
    }

    cout<<endl;
    ROS_INFO("Push: got a plan");
    cout<<endl;

    nav_msgs::Path pushing_path=srvPlan.response.plan;

    vector<double> X,Y,TH;

    //cout << "Federicos path end point: " << pushing_path.poses.back().pose.position.x << " " << pushing_path.poses.back().pose.position.y << endl;

    X.push_back(pushing_path.poses[0].pose.position.x);
    Y.push_back(pushing_path.poses[0].pose.position.y);
    TH.push_back(tf::getYaw(pushing_path.poses[0].pose.orientation));

   //getting coordinates separately

    for(size_t i = 0; i < pushing_path.poses.size(); i++) {
        X.push_back((pushing_path.poses[i].pose.position.x+pushing_path.poses[i-1].pose.position.x)/2);
        Y.push_back((pushing_path.poses[i].pose.position.y+pushing_path.poses[i-1].pose.position.y)/2);
        TH.push_back(tf::getYaw(pushing_path.poses[i].pose.orientation));

        X.push_back(pushing_path.poses[i].pose.position.x);
        Y.push_back(pushing_path.poses[i].pose.position.y);
        TH.push_back(tf::getYaw(pushing_path.poses[i].pose.orientation));
    }
    int path_length = X.size();


    //cout << "calculated path end point: " << X[path_length-1] << " " << Y[path_length-1] << endl;
     nav_msgs::Odometry Odometry;


   try{

    int i=2;
    while ((i<path_length-1)||((i==path_length-1)&&((abs(X[path_length-1]-Ox)>0.1)||(abs(Y[path_length-1]-Oy)>0.1)))){ //error tolerance of 10 cm

                 ros::spinOnce();


                //update of coordinates

                 // /map robot
                // Rx=pose_m_.x;
                // Ry=pose_m_.y;
                // Rth=pose_m_.theta;
                   
                   //odom
                   Odometry = robotino->getOdom();
                    Rx = Odometry.pose.pose.position.x  ; 
                   Ry = Odometry.pose.pose.position.y ;
                   Rth=tf::getYaw(Odometry.pose.pose.orientation);

                // /base_link object

                 try {
                   tf::StampedTransform current_trans;
                   tf_listener.waitForTransform("base_link", goal->object_id, ros::Time::now(), ros::Duration(0.2));
                   tf_listener.lookupTransform("base_link", goal->object_id, ros::Time(0), current_trans);
                   trans = current_trans;
                 } catch (tf::TransformException& ex) {
                   // NOTE: I just ignore the missing object pose and keep using the previous one
                   std::string ns = ros::this_node::getNamespace();
                   std::string node_name = ros::this_node::getName();
                   ROS_ERROR("Push: %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
                 }


              //   Olx=-t.transform.translation.y+offsetX;
               //  Oly=-t.transform.translation.x+offsetY;

                /*transH=Kinect2Base_link(trans.getOrigin().x(),trans.getOrigin().y(),trans.getOrigin().z());
                Olx=transH.pose.position.x;
                Oly=transH.pose.position.y;*/
                Olx=trans.getOrigin().x();
                Oly=trans.getOrigin().y();

                 // /map object
                 transH=Base_link2Odom(Olx,Oly);
                 Ox=transH.pose.position.x;
                 Oy=transH.pose.position.y;

                 // /map object - X[i] and Y[i]

                 // /base_link object wanted
                transH=Odom2Base_link(X[i],Y[i]);
                 Plx=transH.pose.position.x;
                 Ply=transH.pose.position.y;




                 //if object reached to goal
                 if((abs(X[path_length-1]-Ox)<0.10)&&(abs(Y[path_length-1]-Oy)<0.10)){
                     i=path_length;
                     robotino->singleMove(0,0,0.0,0.0,0.0,0);
                     ros::spinOnce();
                     lRate.sleep();

                     ROS_INFO("Push: goal reached");
                     pushResult.result_status = "success";
                     pushServer.setSucceeded(pushResult);
                     stopTracking();

                     cout<<"goal reached"<<endl;
                     cout << "reached position: " << Ox << " " << Oy << " -- ";
                     cout << "actual object target location: " << X[path_length-1] << " " << Y[path_length-1] << endl;
            //       cv_image6.toImageMsg(ros_image);
              //     ImgPub.publish(ros_image);
                 }
                 //if robot got to close to obstacles
                 else if (!robotino->checkDistancesPush(0.06)){ i=path_length;
                     i=path_length;
                     robotino->singleMove(0,0,0.0,0.0,0.0,0);
                     ros::spinOnce();
                     ROS_INFO("Push: obstacle");
                     pushResult.result_status = "failure";
                     pushServer.setAborted(pushResult);
                     stopTracking();
                //   cv_image8.toImageMsg(ros_image);


                     cout<<"obstacle here"<<endl;
                 }
                 else{



                 //check if end of control sequence reached

                 if (i==path_length-1){
                     ROS_INFO("Push: recalculating plan");
                     cout<<endl;
                    // cout<<"recalculating plan"<<endl;
                     int pom=path_length-i;
                     double relx, rely;
                     relx=100; rely=100;
                     for(int j=0; j<path_length-1; j++)
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

                 vOlx.push_back(Ox);
                 vOly.push_back(Oy);

                  if(i>7){
                       bool lost=1;
                       for (int k=1;k<6;k++){
                       if ((vOlx[i-k]!=Ox)||(vOly[i-k]!=Oy)) lost=0;
                       }
                         if(lost){
                         robotino->singleMove(0, 0, 0, 0, 0, 0);
                         lRate.sleep();
                         ros::spinOnce();

                         ROS_INFO("Push: object lost");
                         cout<<endl;
                         pushResult.result_status = "failure";
                         pushServer.setSucceeded(pushResult);
                         stopTracking();
                  //     cv_image7.toImageMsg(ros_image);
                          break;
                    }
                  }

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
                 if (isnan(aORP))aORP=0;

                 aR2O=atan2(Oy-Ry,Ox-Rx);
                 if(aR2O>3.14)aR2O=aR2O-3.14;
                 if(aR2O<-3.14)aR2O=aR2O+3.14;
                 if (isnan(aR2O))aR2O=0;

                 aO2P=atan2(Y[i]-Oy,X[i]-Ox);
                 if(aO2P>3.14)aO2P=aO2P-3.14;
                 if(aO2P<-3.14)aO2P=aO2P+3.14;
                 if(isnan(aO2P))aO2P=0;

                   if(dR2O>0.5){
                         robotino->singleMove(0, 0, 0, 0, 0, 0);
                         lRate.sleep();
                         ros::spinOnce();

                         ROS_INFO("Push: object lost");
                         cout<<endl;
                         pushResult.result_status = "failure";
                         pushServer.setSucceeded(pushResult);
                         stopTracking();
                  //     cv_image7.toImageMsg(ros_image);
                          break;
                    }



                /*cout<<endl<<"object l "<<Olx<<"   "<<Oly<<endl;
                cout<<" plx "<<Plx<<" Ply "<< Ply<<endl;
                cout<<"robot "<<Rx<<"   "<<Ry<<"  "<<Rth<<"   "<<endl;
                cout<<"object "<<Ox<<"   "<<Oy<<endl;
                cout<<"wanted "<<X[i]<<"   "<<Y[i]<<endl;*/

                   
                 double da;

                     /*if ((aO2P<0) && (sgn(aO2P)!=sgn(Rth)))Vth=0.3*(6.28+aO2P-Rth);
                     else if ((aO2P>0) && (sgn(aO2P)!=sgn(Rth)))Vth=0.3*(aO2P-6.28-Rth);
                     else Vth=0.3*(aO2P-Rth);*/

                     if ((aO2P<0) && (sgn(aO2P)!=sgn(Rth)))da=(6.28+aO2P-Rth);
                     else if ((aO2P>0) && (sgn(aO2P)!=sgn(Rth)))da=(aO2P-6.28-Rth);
                     else da=(aO2P-Rth);


                     
               // if ((((Oly<0)&&(Ply>Oly)||(Oly>0)&&(Ply<Oly))||(Plx<-0.02))&&(aORP>0.4)){
                 if (((((Oly<0)&&(Ply>Oly)||(Oly>0)&&(Ply<Oly)))&&(aORP>0.3))&&(Olx<(d+0.250))&&(dR2O<d+0.5)){
                     // cout<<"action 1"<<endl;
                     // cout<<"difference "<< Ply-Oly<<" aORP "<<aORP<<" vx "<<Vx<<endl;

                     va1.push_back(Oly-Ply);
                     double va1_mean=std::accumulate(va1.begin(), va1.end(),0)/va1.size();
                     double va1_meanE=std::inner_product(va1.begin(), va1.end(),va1.begin(), 0.0)/va1.size();
                     double va1_var=va1_meanE-va1_mean*va1_mean;
                     double K1=10e-04;

                   ROS_INFO("Push: action 1");
                  Vx=0; Vy=1.5*(Oly-Ply); Vth=0;
                     Vx=0; Vy=(Oly-Ply)*K1/va1_var;Vth=0;

                   //Vx=0; Vy=0.8*(Oly-Ply); Vth=0;
                    i=i-1;
                    //v_image1.toImageMsg(ros_image);
                     }

                 // else if((abs(Rth-aO2P)>0.5)&&(dO2P>0.1)&&(abs(Oly)>0.03)&&(abs(Rth-aR2O)<1.2)&&(abs(Ply-Oly)>0.08)){
                 else if(((abs(da)>0.3)&&(dO2P>0.1)&&(abs(Olx)>0.03)&&(abs(Rth-aR2O)<0.8)&&(abs(Plx-Olx)>0.08))&&(Olx<(d+0.25))){
                 //else if((abs(th-aO2P)>0.3)&&(abs(th-aR2O)<1.2)&&(dO2P>0.1)){


                    //cout<< "action 2 "<<endl;
                    //cout<<"aO2P "<<aO2P<<" robot th: "<<Rth<< " difference "<<Rth-aO2P<<" dO2P "<<dO2P<<" th-aR2O "<<Rth-aR2O<< endl;

                     Vx=0; Vy=0;
                     //double da;

                     /*if ((aO2P<0) && (sgn(aO2P)!=sgn(Rth)))Vth=0.3*(6.28+aO2P-Rth);
                     else if ((aO2P>0) && (sgn(aO2P)!=sgn(Rth)))Vth=0.3*(aO2P-6.28-Rth);
                     else Vth=0.3*(aO2P-Rth);*/

                     //if ((aO2P<0) && (sgn(aO2P)!=sgn(Rth)))da=(6.28+aO2P-Rth);
                     //else if ((aO2P>0) && (sgn(aO2P)!=sgn(Rth)))da=(aO2P-6.28-Rth);
                     //else da=(aO2P-Rth);

                     va2.push_back(da);


                     double va2_mean=std::accumulate(va2.begin(), va2.end(),0)/va2.size();
                     double va2_meanE=std::inner_product(va2.begin(), va2.end(),va2.begin(), 0.0)/va2.size();
                     double va2_var=va2_meanE-va2_mean*va2_mean;

                     Vth=va2_var*da;
                     Vth=0.3*da;
                     //Vth=0.2*da;
                     ROS_INFO("Push: action 2");
                    // Vth=0.2*da;

                     i=i-1;

                 //   cv_image2.toImageMsg(ros_image);
                }

                 //else if((abs(th-aR2O)>0.1)||(abs(th-aO2P))>0.2){
                else if ((abs(Rth-aR2O)>0.3)&&(dO2P<0.20)){
                    // cout<<"action 3 "<<endl;
                    // cout<<"aR2O"<<aR2O<<" robot th: "<<Rth<< "difference"<<Rth-aR2O<<endl;

                     va3.push_back(aR2O-Rth);

                     double va3_mean=std::accumulate(va3.begin(), va3.end(),0)/va3.size();
                     double va3_meanE=std::inner_product(va3.begin(), va3.end(),va3.begin(), 0.0)/va3.size();
                     double va3_var=va3_meanE-va3_mean*va3_mean;

                     Vth=va3_var*aR2O-Rth;
                     Vx=0; Vy=0; Vth=0.3*(aR2O-Rth);

                    // Vx=0; Vy=0; Vth=0.2*(aR2O-Rth);
                     ROS_INFO("Push: action 3");
                     i=i-1;

                   //v_image3.toImageMsg(ros_image);
                }


               //lse if ((abs(Oly)>0.10)||(abs(Oly)>0.05)&&((Oly<0)&&(Ply>Oly)||(Oly>0)&&(Ply<Oly))||(Olx>(d+0.10))){
   else if ((abs(Oly)>0.10)||(abs(Oly)>0.05)&&((Oly<0)&&(Ply>Oly)||(Oly>0)&&(Ply<Oly))||(Olx>(d+0.08))){


                     //cout<<"action 4 "<<endl;
                     //cout<<"x "<<0.3*(abs(Olx)-d)<<" y "<<Oly<<endl;
                     va4x.push_back(abs(Olx)-d);
                     va4y.push_back(Oly);

                     double va4x_mean=std::accumulate(va4x.begin(), va4x.end(),0)/va4x.size();
                     double va4x_meanE=std::inner_product(va4x.begin(), va4x.end(),va4x.begin(), 0.0)/va4x.size();
                     double va4x_var=va4x_meanE-va4x_mean*va4x_mean;

                     double va4y_mean=std::accumulate(va4y.begin(), va4y.end(),0)/va4y.size();
                     double va4y_meanE=std::inner_product(va4y.begin(), va4y.end(),va4y.begin(), 0.0)/va4y.size();
                     double va4y_var=va4y_meanE-va4y_mean*va4y_mean;

                     double K4=1.5e-03;

                     Vx=va4x_var*(abs(Olx)-d); Vy=Oly*K4/va4y_var; Vth=0;



                     //Vx=0.2*(abs(Olx)-d); Vy=Oly; Vth=0;
                     Vx=0.3*(abs(Olx)-d); Vy=Oly; Vth=0;

                    // Vx=0.2*(abs(Olx)-d); Vy=Oly; Vth=0;
                    ROS_INFO("Push: action 4");

                     i=i-1;
                     //_image4.toImageMsg(ros_image);

                }

                else{
                //    cout<<"here 0"<<endl;
                    vElx.push_back(errXl);
                    vEly.push_back(errYl);
                    ROS_INFO("Push: action main");


                    double ex_mean=std::accumulate(vElx.begin(), vElx.end(),0)/vElx.size();
                    double ey_mean=std::accumulate(vEly.begin(), vEly.end(),0)/vEly.size();
                    double ex_meanE=std::inner_product(vElx.begin(), vElx.end(),vElx.begin(), 0.0)/vElx.size();
                    double ey_meanE=std::inner_product(vEly.begin(), vEly.end(),vEly.begin(), 0.0)/vEly.size();
                    double ex_var=ex_meanE-ex_mean*ex_mean;
                    double ey_var=ey_meanE-ey_mean*ey_mean;

                   // Vx=0.004*(errXl/ex_var +0.1*dErrXl/ex_var);
                  //  Vy=0.002*(errYl/ey_var +0.1*dErrYl/ey_var);
                    Vx=2*(errXl*ex_var +0.1*dErrXl*ex_var);
                    Vy=2*(errYl*ey_var +0.1*dErrYl*ey_var);

                  //Vx=0.4*errXl+0.1*dErrXl;
                  // Vy=0.2*errYl+0.1*dErrYl;
                    Vth=0;
                    //cv_image5.toImageMsg(ros_image);

                   // cout<<"Vx "<<Vx<<" Vy "<<Vy<<endl;

                }



               if (i<path_length-1)i++;
              // else{
                //   ROS_INFO(
                 //  cout<<"end"<<endl<<"current local errors x: "<<errXl<<" y:" <<errYl<<endl;
              // }

               if (Vx>0.2)Vx=0.2;
               if (Vx<-0.2)Vx=-0.2;
               if (Vy>0.2)Vy=0.2;
               if (Vy<-0.2)Vy=-0.2;
               if (Vth>0.2)Vth=0.2;
               if (Vth<-0.2)Vth=-0.2;

                robotino->singleMove(Vx,Vy,0.0,0.0,0.0,Vth);
                // ImgPub.publish(ros_image);
                lRate.sleep();
               // pushFeedback.percent_completed=i*100/path_length;
               // pushServer.publishFeedback(pushFeedback.percent_completed);
                ros::spinOnce();



                //ROS_INFO_STREAM("loop");
               // cout<<endl;

               // cout<<endl<<" local position object x: "<<Olx<<" y "<<Oly<<endl<<" map position object x: "<<Ox<<" y "<<Oy<<endl<<" Robot map position " <<" x "<<Rx<<" y "<<Ry<<endl<<endl;

                 }
    }
   }
   catch(...){
       pushResult.result_status = "failure";
       pushServer.setAborted(pushResult);
      //  cv_image8.toImageMsg(ros_image);
       stopTracking();
   }


    /* for(int i = 1; i <= 10; ++i) {

        rate.sleep();
        PushFeedback.percent_completed = i * 10;
        pushServer.publishFeedback(PushFeedback);

    }*/

    tilt_msg.data = 0.60;
    TiltPub.publish(tilt_msg);
    ros::spinOnce();


   // pushResult.result_status = "success";
   // pushServer.setSucceeded(pushResult);

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
        tf_listener.waitForTransform("/map","/base_link", ros::Time::now(), ros::Duration(0.2));
        tf_listener.transformPose("/base_link",Emap,Eloc);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("Push: %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

    }

    return Eloc;
}




geometry_msgs::PoseStamped Kinect2Base_link(double x, double y, double z){

    geometry_msgs::PoseStamped Ekin, Eloc;
    tf::TransformListener tf_listener;

    Ekin.pose.position.x=x;
    Ekin.pose.position.y=y;
    Ekin.pose.position.z=z;
    Ekin.pose.orientation.x=0;
    Ekin.pose.orientation.y=0;
    Ekin.pose.orientation.z=0;
    Ekin.pose.orientation.w=1;
    Ekin.header.frame_id="/kinect_rgb_optical_frame";
    try {
        tf_listener.waitForTransform("/kinect_rgb_optical_frame","/base_link", ros::Time::now(), ros::Duration(0.2));
        tf_listener.transformPose("/base_link",Ekin,Eloc);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("Push: %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

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
    Emap.header.frame_id="/base_link";
    try {
        tf_listener.waitForTransform("/base_link","/map", ros::Time::now(), ros::Duration(0.2));
        tf_listener.transformPose("/map",Emap,Eloc);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

    }

    return Eloc;
}

geometry_msgs::PoseStamped odom2Map(double x, double y){

    geometry_msgs::PoseStamped Emap, Eloc;
    tf::TransformListener tf_listener;

    Emap.pose.position.x=x;
    Emap.pose.position.y=y;
    Emap.pose.position.z=0;
    Emap.pose.orientation.x=0;
    Emap.pose.orientation.y=0;
    Emap.pose.orientation.z=0;
    Emap.pose.orientation.w=1;
    Emap.header.frame_id="/base_link";
    try {
        tf_listener.waitForTransform("/odom","/map", ros::Time::now(), ros::Duration(0.2));
        tf_listener.transformPose("/map",Emap,Eloc);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

    }

    return Eloc;
}

bool startTracking(std::string object_id) {
    squirrel_object_perception_msgs::StartObjectTracking srvStartTrack;
    srvStartTrack.request.object_id.data = object_id;
    return ros::service::call("/squirrel_start_object_tracking", srvStartTrack);
}

bool stopTracking() {
    squirrel_object_perception_msgs::StopObjectTracking srvStopTrack;
    return ros::service::call("/squirrel_stop_object_tracking", srvStopTrack);
}

geometry_msgs::PoseStamped Base_link2Odom(double x, double y){

    geometry_msgs::PoseStamped Emap, Eloc;
    tf::TransformListener tf_listener;

    Emap.pose.position.x=x;
    Emap.pose.position.y=y;
    Emap.pose.position.z=0;
    Emap.pose.orientation.x=0;
    Emap.pose.orientation.y=0;
    Emap.pose.orientation.z=0;
    Emap.pose.orientation.w=1;
    Emap.header.frame_id="/base_link";
    try {
        tf_listener.waitForTransform("/base_link","/odom", ros::Time::now(), ros::Duration(0.2));
        tf_listener.transformPose("/odom",Emap,Eloc);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

    }

    return Eloc;
}


geometry_msgs::PoseStamped Odom2Base_link(double x, double y){

    geometry_msgs::PoseStamped Emap, Eloc;
    tf::TransformListener tf_listener;

    Emap.pose.position.x=x;
    Emap.pose.position.y=y;
    Emap.pose.position.z=0;
    Emap.pose.orientation.x=0;
    Emap.pose.orientation.y=0;
    Emap.pose.orientation.z=0;
    Emap.pose.orientation.w=1;
    Emap.header.frame_id="/odom";
    try {
        tf_listener.waitForTransform("/odom","/base_link", ros::Time::now(), ros::Duration(0.2));
        tf_listener.transformPose("/base_link",Emap,Eloc);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("Push: %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

    }

    return Eloc;
}

