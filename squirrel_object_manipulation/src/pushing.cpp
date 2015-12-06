#include <squirrel_object_manipulation/pushing.hpp>



using namespace std;

bool startTracking(std::string object_id);
bool stopTracking();


PushAction::PushAction(const std::string std_PushServerActionName) :
    pushServer(nh, std_PushServerActionName, boost::bind(&PushAction::executePush, this, _1), false),
    private_nh("~")
{
    pushServer.start();
    private_nh.param<std::string>("pose_topic", pose_topic_,"/squirrel_localizer_pose");

    pose_sub_ = nh.subscribe(pose_topic_, 2, &PushAction::updatePose, this);
    robotino = new RobotinoControl(nh);
}

PushAction::~PushAction() {
    delete robotino;
}

void PushAction::executePush(const squirrel_manipulation_msgs::PushGoalConstPtr &goal) {

    //move camera for vision
    robotino->moveTilt(0.66);

    ROS_INFO("Push: (push up action) started push up of %s for manipulation",  goal->object_id.c_str());


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
    cout<< "1"<<endl;
    tf::TransformListener tf_listener;
    tf::StampedTransform trans;

    try {
        tf_listener.waitForTransform("base_link", goal->object_id, ros::Time::now(), ros::Duration(10.0));
        tf_listener.lookupTransform("base_link", goal->object_id, ros::Time(0), trans);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("Push: %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
        pushResult.result_status = "failure";
        pushServer.setAborted(pushResult);
        stopTracking();
        return;
    }
    cout<< "2"<<endl;

    ros::Rate lRate(10);

    double d=0.23;
    double Rx, Ry, Rth, Ox, Oy, Olx, Oly, Plx, Ply;
    double Vx, Vy, Vth;
    double dR2O, dR2P, dO2P, aORP, aR2O, aO2P, errXl, errYl, dErrXl, dErrYl;
    vector<double> vOlx, vOly, Elx, Ely, va1, va2, va3, va4x, va4y,  vElx,  vEly;


    ros::spinOnce();

    squirrel_rgbd_mapping_msgs::GetPushingPlan srvPlan;

    ROS_INFO("Push: getting object position \n");

    Olx=trans.getOrigin().x();
    Oly=trans.getOrigin().y();
    cout<<"Push: Object position in base_link x: "<<Olx<<" y "<< Oly << endl;

    geometry_msgs::PoseStamped transH;

    // /map object
    transH=Base_link2Map(Olx,Oly);
    Ox=transH.pose.position.x;
    Oy=transH.pose.position.y;

    cout<<"Push: Object position in map x: "<<Ox<<" y "<<Oy<<endl;

    // Getting pushing plan

    ROS_INFO("Push: requesting plan \n");
    srvPlan.request.start.x = Ox;
    srvPlan.request.start.y = Oy;
    srvPlan.request.start.theta = pose_m_.theta;

    cout << "received object target position: " << goal->pose.position.x << " "  << goal->pose.position.y << "\n";
    srvPlan.request.goal.x = goal->pose.position.x;
    srvPlan.request.goal.y = goal->pose.position.y;
    srvPlan.request.goal.theta = tf::getYaw(goal->pose.orientation);

    geometry_msgs::Point32 p1, p2, p3, p4;

    p1.x = 0.20; p1.y = -0.20;
    p2.x = 0.20; p1.y = 0.20;
    p3.x = -0.20; p1.y = -0.20;
    p4.x = -0.20; p1.y = 0.20;

    srvPlan.request.object.points.push_back(p1);
    srvPlan.request.object.points.push_back(p2);
    srvPlan.request.object.points.push_back(p3);
    srvPlan.request.object.points.push_back(p4);

    if ( ros::service::call("/getPushingPlan", srvPlan) ) {
        if ( srvPlan.response.plan.poses.empty() ) {
            ROS_WARN("Push:got an empty plan");
        } else {
            BOOST_ASSERT_MSG( srvPlan.response.plan.header.frame_id == "/map" ||
                              srvPlan.response.plan.header.frame_id == "map" ,
                              "returned path is not in '/odom' frame");
            ROS_INFO("Push: got a path for pushing");
        }
    } else {
        ROS_ERROR("Push: unable to communicate with /getPushingPlan");
        pushResult.result_status = "failure";
        pushServer.setAborted(pushResult);
        stopTracking();
    }

    ROS_INFO("Push: got a plan \n");

    nav_msgs::Path pushing_path=srvPlan.response.plan;

    vector<double> X,Y,TH;

    cout << "(Push) nav path end point: " << pushing_path.poses.back().pose.position.x << " " << pushing_path.poses.back().pose.position.y << endl;

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


    cout << "Push: calculated path end point: " << X[path_length-1] << " " << Y[path_length-1] << endl;

   //main control loop

    try{

        int i=0;
        while ((i<path_length-1)||((i==path_length-1)&&((abs(X[path_length-1]-Ox)>0.1)||(abs(Y[path_length-1]-Oy)>0.1)))){ //error tolerance of 10 cm

            ros::spinOnce();

            //update of coordinates

            // /map robot
            Rx=pose_m_.x;
            Ry=pose_m_.y;
            Rth=pose_m_.theta;

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

            Olx=trans.getOrigin().x();
            Oly=trans.getOrigin().y();

            // /map object
            transH=Base_link2Map(Olx,Oly);
            Ox=transH.pose.position.x;
            Oy=transH.pose.position.y;


            // /base_link object wanted
            transH=Map2Base_link(X[i],Y[i]);
            Plx=transH.pose.position.x;
            Ply=transH.pose.position.y;


            //if object reached to goal
            if((abs(X[path_length-1]-Ox)<0.08)&&(abs(Y[path_length-1]-Oy)<0.08)){
                i=path_length;
                robotino->singleMove(0,0,0.0,0.0,0.0,0);
                ros::spinOnce();
                lRate.sleep();

                ROS_INFO("Push: goal reached");
                pushResult.result_status = "success";
                pushServer.setSucceeded(pushResult);
                stopTracking();

                cout <<"goal reached"<<endl;
                cout << "reached position: " << Ox << " " << Oy << " -- ";
                cout << "actual object target location: " << X[path_length-1] << " " << Y[path_length-1] << endl;

            }

            // check if robot got to close to obstacles
            else if (!robotino->checkDistancesPush(0.06)){ i=path_length;
                i=path_length;
                robotino->singleMove(0,0,0.0,0.0,0.0,0);
                ros::spinOnce();
                ROS_INFO("Push: obstacle");
                pushResult.result_status = "failure";
                pushServer.setAborted(pushResult);
                stopTracking();


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

                //check if object lost

                vOlx.push_back(Ox);
                vOly.push_back(Oy);

                if(i>5){
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

                // if ((((Oly<0)&&(Ply>Oly)||(Oly>0)&&(Ply<Oly))||(Plx<-0.02))&&(aORP>0.4)){
                if ((((Oly<0)&&(Ply>Oly)||(Oly>0)&&(Ply<Oly)))&&(aORP>0.3)){

                    va1.push_back(Oly-Ply);
                    double va1_mean=std::accumulate(va1.begin(), va1.end(),0)/va1.size();
                    double va1_meanE=std::inner_product(va1.begin(), va1.end(),va1.begin(), 0.0)/va1.size();
                    double va1_var=va1_meanE-va1_mean*va1_mean;
                    double K1=10e-04;

                    // Vx=0; Vy=1.5*(Oly-Ply); Vth=0;
                    Vx=0; Vy=(Oly-Ply)*K1/va1_var;Vth=0;
                    // Vx=0; Vy=0.8*(Oly-Ply); Vth=0;
                    i=i-1;

                }

                // else if((abs(Rth-aO2P)>0.5)&&(dO2P>0.1)&&(abs(Oly)>0.03)&&(abs(Rth-aR2O)<1.2)&&(abs(Ply-Oly)>0.08)){
                else if((abs(Rth-aO2P)>0.3)&&(dO2P>0.1)&&(abs(Olx)>0.03)&&(abs(Rth-aR2O)<1.2)&&(abs(Plx-Olx)>0.08)){

                    Vx=0; Vy=0;
                    double da;

                    if ((aO2P<0) && (boost::math::sign(aO2P)!=boost::math::sign(Rth)))da=(6.28+aO2P-Rth);
                    else if ((aO2P>0) && (boost::math::sign(aO2P)!=boost::math::sign(Rth)))da=(aO2P-6.28-Rth);
                    else da=(aO2P-Rth);
                    va2.push_back(da);
                    double va2_mean=std::accumulate(va2.begin(), va2.end(),0)/va2.size();
                    double va2_meanE=std::inner_product(va2.begin(), va2.end(),va2.begin(), 0.0)/va2.size();
                    double va2_var=va2_meanE-va2_mean*va2_mean;

                    Vth=va2_var*da;
                    Vth=0.3*da;
                    // Vth=0.2*da;

                    i=i-1;

                }

                //else if((abs(th-aR2O)>0.1)||(abs(th-aO2P))>0.2){
                else if ((abs(Rth-aR2O)>0.2)&&(dO2P<0.1)){

                    va3.push_back(aR2O-Rth);

                    double va3_mean=std::accumulate(va3.begin(), va3.end(),0)/va3.size();
                    double va3_meanE=std::inner_product(va3.begin(), va3.end(),va3.begin(), 0.0)/va3.size();
                    double va3_var=va3_meanE-va3_mean*va3_mean;

                    Vth=va3_var*aR2O-Rth;
                    //Vx=0; Vy=0; Vth=0.3*(aR2O-Rth);
                    //Vx=0; Vy=0; Vth=0.2*(aR2O-Rth);

                    i=i-1;

                }


                else if ((abs(Oly)>0.10)||(abs(Oly)>0.05)&&((Oly<0)&&(Ply>Oly)||(Oly>0)&&(Ply<Oly))||(Olx>(d+0.10))){
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
                    Vx=0.3*(abs(Olx)-d); Vy=Oly; Vth=0;

                    i=i-1;

                }

                else{

                    vElx.push_back(errXl);
                    vEly.push_back(errYl);

                    double ex_mean=std::accumulate(vElx.begin(), vElx.end(),0)/vElx.size();
                    double ey_mean=std::accumulate(vEly.begin(), vEly.end(),0)/vEly.size();
                    double ex_meanE=std::inner_product(vElx.begin(), vElx.end(),vElx.begin(), 0.0)/vElx.size();
                    double ey_meanE=std::inner_product(vEly.begin(), vEly.end(),vEly.begin(), 0.0)/vEly.size();
                    double ex_var=ex_meanE-ex_mean*ex_mean;
                    double ey_var=ey_meanE-ey_mean*ey_mean;

                    Vx=2*(errXl*ex_var +0.1*dErrXl*ex_var);
                    Vy=2*(errYl*ey_var +0.1*dErrYl*ey_var);
                    Vth=0;

                }



                if (i<path_length-1)i++;
                else{
                    cout<<"(Push) end"<<endl<<"current local errors x: "<<errXl<<" y:" <<errYl<<endl;
                }

                if (Vx>0.2)Vx=0.2;
                if (Vx<-0.2)Vx=-0.2;
                if (Vy>0.2)Vy=0.2;
                if (Vy<-0.2)Vy=-0.2;
                if (Vth>0.2)Vth=0.2;
                if (Vth<-0.2)Vth=-0.2;

                robotino->singleMove(Vx,Vy,0.0,0.0,0.0,Vth);

                lRate.sleep();
                // pushFeedback.percent_completed=i*100/path_length;
                // pushServer.publishFeedback(pushFeedback.percent_completed);
                ros::spinOnce();


            }
        }
    }
    catch(...){
        pushResult.result_status = "failure";
        pushServer.setAborted(pushResult);
        stopTracking();
    }


    robotino->moveTilt(0.60);
    ros::spinOnce();

}


int main(int argc, char** argv) {

    ros::init(argc, argv, "manipulation");

    PushAction push(PUSH_NAME);
    ros::spin();

    return 0;

}

void PushAction::updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg )
{
  pose_m_.x = pose_msg->pose.pose.position.x;
  pose_m_.y = pose_msg->pose.pose.position.y;
  pose_m_.theta = tf::getYaw(pose_msg->pose.pose.orientation);
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

