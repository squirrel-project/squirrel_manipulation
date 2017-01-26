#include <squirrel_handovers/handover.hpp>


using namespace std;
using namespace arma;
using namespace kukadu;

HandoverAction::HandoverAction(const std::string std_HandoverServerActionName) :
    handoverServer(nh, std_HandoverServerActionName, boost::bind(&HandoverAction::executeHandover, this, _1), false),
    private_nh("~")
{
    private_nh.param("base_frame", base_frame_, std::string("/base_link"));
    private_nh.param("wrist_frame", wrist_frame_, std::string("/arm_link5"));
    private_nh.param("tuw_robotino", tuw_robotino, std::string("tuw-robotino2"));
    private_nh.param("uibk_robotino", uibk_robotino, std::string("uibk-robotino2-sh"));
    private_nh.param("robot", robot, std::string("uibk-robotino2-sh"));

    sub_h = nh.subscribe(SENSOR_TOPIC, 1, &HandoverAction::sensorReadCallbackWrist,this);

    if(robot == tuw_robotino){
        sub_f = nh.subscribe(FINGERTIP_TOPIC, 1, &HandoverAction::sensorReadCallbackFingers, this);
    }

    handoverServer.start();
    ROS_INFO("(Handover) server started \n");

}

void HandoverAction::executeHandover(const squirrel_manipulation_msgs::HandoverGoalConstPtr &goal) {


    ROS_INFO("(Handover) action started \n");
    cout<<endl;

    cout<<"(Handover) action type"<<goal->action_type<<endl;
    cout<<"(Handover) handover type"<<goal->handover_type<<endl;

    //hand
    squirrel_manipulation_msgs::SoftHandGrasp graspService;
    graspService.request.position = 0.9;
    squirrel_manipulation_msgs::SoftHandGrasp releaseService;
    releaseService.request.position = 0.0;

    actionlib::SimpleActionClient<kclhand_control::ActuateHandAction> kclhandGraspActionClient("hand_controller/actuate_hand", true);
    if (robot == tuw_robotino){
        kclhandGraspActionClient.waitForServer();}
    kclhand_control::ActuateHandActionGoal graspServiceKCL;
    graspServiceKCL.goal.command = 1.0;
    graspServiceKCL.goal.force_limit = 1.0;
    kclhand_control::ActuateHandActionGoal releaseServiceKCL;
    releaseServiceKCL.goal.command = 0.0;
    releaseServiceKCL.goal.force_limit = 1.0;


    sleep (1);


    //arm control

    ROS_INFO("(handover) setting up control queue");
    cout <<  endl;
    auto robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", nh));

    ROS_INFO("(handover) creating moveit kinematics instance");
    cout << endl;
    vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    auto mvKin = make_shared<MoveItKinematics>(robotinoQueue, nh, "robotino", controlledJoints, "arm_link5");

    robotinoQueue->setKinematics(mvKin);
    robotinoQueue->setPathPlanner(mvKin);

    ROS_INFO("(handover) starting queue");
    cout << endl;
    auto realLqThread = robotinoQueue->startQueue();

    cout << "(handover) switching to impedance mode if it is not there yet" << endl;
    if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    }

    auto start = stdToArmadilloVec({0.0, 0.0, 0.0, 1.0, -0.5, 1.4, 1.0, 1.6});
    auto end = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});


    bool handover_success_ = false;
    stage = 0;
    
    std::string take ("take");
    std::string give ("give");    

    if(take.compare(goal->action_type.c_str())==0){
        sleep(1);
        auto firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;

        ROS_INFO("(handover) going to initial pose with the open hand");
        stage = 0;
        cout << "(handover) current stage "<<stage<<endl;
        robotinoQueue->jointPtp(start);


        stage = 1; // initial pose with the open hand
        cout << "(handover) current stage "<<stage<<endl;
        //firstJoints = robotinoQueue->getCurrentJoints().joints;
        //cout << "(handover) current robot state: " << firstJoints.t() << endl;


        ROS_INFO("(handover) going to the handover pose with the open hand");
        stage = 2;
        cout << "(handover) current stage "<<stage<<endl;

        robotinoQueue->jointPtp(end);

        ROS_INFO("(handover) waiting to grasp the object");
        stage = 3;
        bool grasp_value = true; //detect object

        stage = 4; //grasping the object
        cout << "(handover) current stage "<<stage<<endl;


        if(grasp_value){
            // stage = 4; //grasping the object
            cout<<"OK"<<endl;

            if (robot == uibk_robotino){

                if ( ros::service::call(HAND_SERVICE, graspService) ){
                    ROS_INFO("(handover) HAND Grasped!");
                }else{
                    ROS_ERROR("handover) FAILED to Graps!");
                }
            }
            else if (robot == tuw_robotino){
                kclhandGraspActionClient.sendGoal(graspServiceKCL.goal);
                kclhandGraspActionClient.waitForResult(ros::Duration(5.0));
                if (kclhandGraspActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("(handover) HAND Grasped!");
                }else{
                    ROS_ERROR("handover) FAILED to Graps!");
                }
            }
        }
        cout << "(handover) current stage "<<stage<<endl;

        ROS_INFO("(handover) going to initial pose with the closed hand");
        stage = 5;
        cout << "(handover) current stage "<<stage<<endl;
        robotinoQueue->jointPtp(start);
    }
    else if(take.compare(goal->action_type.c_str()) ==0){

        ROS_INFO("(handover) going to the handover pose with the closed hand");
        stage = 6; // initial pose with the closed hand
        cout << "(handover) current stage "<<stage<<endl;

        robotinoQueue->jointPtp(end);

        ROS_INFO("(handover) waiting to release the object");
        stage = 7;
        cout << "(handover) current stage "<<stage<<endl;
        bool release_value = true;//here detection
        stage = 8  ; //releasing the object
        cout << "(handover) current stage "<<stage<<endl;

        if(release_value){
            cout<<"OK"<<endl;

            if (robot == uibk_robotino){

                if ( ros::service::call(HAND_SERVICE, releaseService) ){
                    ROS_INFO("(handover) HAND Released!");
                }else{
                    ROS_ERROR("handover) FAILED to Release!");
                }
            }
            else if (robot == tuw_robotino){
                kclhandGraspActionClient.sendGoal(releaseServiceKCL.goal);
                kclhandGraspActionClient.waitForResult(ros::Duration(5.0));
                if (kclhandGraspActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("(handover) HAND Released!");
                }else{
                    ROS_ERROR("handover) FAILED to Release!");
                }
            }
        }

        stage = 9;
        cout << "(handover) current stage "<<stage<<endl;

        ROS_INFO("(handover) going to initial pose with the open hand");
        robotinoQueue->jointPtp(start);

    }
    else{
        ROS_ERROR(" Handover: Handover action unknown");
        cout<<endl;
    }

    ROS_INFO(" Handover: Handover sequence finished.");
    cout<<endl;
    


    handover_success_ = true;

    if(handover_success_){
        handoverResult.result_status = "success";
        handoverServer.setSucceeded(handoverResult);
        ROS_INFO(" Handover: Sucessfuly finished ");
        cout<< endl;
    }
    else{
        handoverResult.result_status = "failure";
        handoverServer.setAborted(handoverResult);
        ROS_INFO(" Handover: failed ");
        cout<< endl;}
}

HandoverAction::~HandoverAction() {


}

vector<double> HandoverAction::projectVectors(double vecX,double vecY,double vecZ,double alpha,double beta,double gamma){
    vector<double> newVec;
    double i= vecX * (cos(beta)*cos(gamma)) + vecY *(cos(gamma)* sin(alpha)* sin(beta) - cos(alpha)*sin(gamma)) + vecZ *(cos(alpha)*cos(gamma)*sin(beta) +sin(alpha)*sin(gamma));
    double j= vecX * (cos(beta)*sin(gamma)) + vecY *(cos(alpha)* cos(gamma) + sin(alpha)*sin(gamma)*sin(beta)) + vecZ *(-1*cos(gamma)*sin(alpha) +cos(alpha)*sin(beta)*sin(gamma));
    double k= vecX * (-1*sin(beta)) + vecY *(cos(beta)* sin(alpha)) + vecZ *(cos(alpha)*cos(beta)); // ref: roll-x-alpha pitch-y-beta yaw-z-gamma
    newVec.push_back(i);
    newVec.push_back(j);
    newVec.push_back(k);
    return newVec;

}
std::vector<double> HandoverAction::projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose){
    tf::Quaternion quat(currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z,currentPose.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    //Fix senser miss-alignment in respect to end effector and wrong sensor roll yaw conventions!!
    vector<double> temp1 = projectVectors(readings.at(0),readings.at(1) ,readings.at(2),0.0,0.0,SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR);
    vector<double> temp2 = projectVectors(readings.at(3),readings.at(4) ,readings.at(5),0.0,0.0, M_PI + SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR);
    //Project based on joint states
    vector<double> res1 = projectVectors(temp1.at(0),temp1.at(1) ,temp1.at(2),roll,pitch,yaw);
    vector<double> res2 = projectVectors(temp2.at(0),temp2.at(1) ,temp2.at(2),roll,pitch,yaw);
    res1.insert(res1.end(),res2.begin(),res2.end());

    return res1;
}

geometry_msgs::Pose HandoverAction::tf_stamped2pose(tf::StampedTransform tf_in){
    geometry_msgs::Pose Emap;

    Emap.position.x = tf_in.getOrigin().x();
    Emap.position.y = tf_in.getOrigin().y();
    Emap.position.z = tf_in.getOrigin().z();;
    Emap.orientation.x = tf_in.getRotation().x();
    Emap.orientation.y = tf_in.getRotation().y();
    Emap.orientation.z = tf_in.getRotation().z();;
    Emap.orientation.w = tf_in.getRotation().w();;

    return Emap;
}

void HandoverAction::sensorReadCallbackWrist(std_msgs::Float64MultiArray msg){
    wrist_sensor_values_ = msg.data;
   cout<<wrist_sensor_values_ <<endl;   
//    wrist_sensor_values_.at(0) = msg.data.at(0);
    //    wrist_sensor_values_.at(1) = msg.data.at(1);
    //    wrist_sensor_values_.at(2) = msg.data.at(2);
    //    wrist_sensor_values_.at(3) = msg.data.at(3);
    //    wrist_sensor_values_.at(4) = msg.data.at(4);
    //    wrist_sensor_values_.at(5) = msg.data.at(5);

}

void HandoverAction::sensorReadCallbackFingers(std_msgs::Float64MultiArray msg){

    fingertip_sensor_values_ = msg.data;
    //    fingertip_sensor_values_.at(0) = msg.data.at(0);
    //    fingertip_sensor_values_.at(1) = msg.data.at(1);
    //    fingertip_sensor_values_.at(2) = msg.data.at(2);
    //    fingertip_sensor_values_.at(3) = msg.data.at(3);
    //    fingertip_sensor_values_.at(4) = msg.data.at(4);
    //    fingertip_sensor_values_.at(5) = msg.data.at(5);
    //    fingertip_sensor_values_.at(6) = msg.data.at(6);
    //    fingertip_sensor_values_.at(7) = msg.data.at(7);
    //    fingertip_sensor_values_.at(8) = msg.data.at(8);
    //    fingertip_sensor_values_.at(9) = msg.data.at(9);
    //    fingertip_sensor_values_.at(10) = msg.data.at(10);
    //    fingertip_sensor_values_.at(11) = msg.data.at(11);
    //    fingertip_sensor_values_.at(12) = msg.data.at(12);
    //    fingertip_sensor_values_.at(13) = msg.data.at(13);
    //    fingertip_sensor_values_.at(14) = msg.data.at(14);
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "manipulation");

//    HandoverAction handover(HANDOVER_NAME);
//    ros::AsyncSpinner spinner(10); spinner.start();
    HandoverAction hadnover(HANDOVER_NAME);
    ros::spin();
    return 0;

}


