#include <vector>
#include <memory>
#include <iostream>
#include <cmath>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <kukadu/kukadu.hpp>
#include <squirrel_manipulation_msgs/SoftHandGrasp.h>
#include <kclhand_control/ActuateHandAction.h>
#include <kclhand_control/ActuateHandActionGoal.h>
#include <actionlib/client/simple_action_client.h>

#define SENSOR_TOPIC "/wrist"
#define FINGERTIP_TOPIC "/fingertips"
#define HAND_SERVICE "/softhand_grasp"

static auto constexpr  SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR = -2.27;


using namespace std;
using namespace arma;
using namespace kukadu;

void sensorReadCallbackWrist(std_msgs::Float64MultiArray msg);
void sensorReadCallbackFingers(std_msgs::Float64MultiArray msg);
void dataStore();
geometry_msgs::Pose tf_stamped2pose(tf::StampedTransform tf_in);


string base_frame_ = "/base_link";
string wrist_frame_ = "/arm_link5";
string robot;
string tuw_robotino = "tuw-robotino2";
string uibk_robotino = "uibk-robotino2-sh";
string path_= "/home/c7031098/catkin_ws/data/";
//string path_ ="/home/c7031098/squirrel_ws_new/data/";
string experiment_;


bool store_set_(false), store_(false), writing_done_(false);
int stage = 100;
int end_task = 100;

boost::mutex sensor_mutex_;
std::vector<double>  robot_joints_, wrist_sensor_values_(6,0.0), fingertip_sensor_values_(15,0.0);
std::vector<double> projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose);



int main(int argc, char** args) {

    ros::init(argc, args, "handover_data_collection");
    ros::NodeHandle node;

    node.param("robot",robot,std::string(uibk_robotino));
    sleep(1);
    boost::thread* data_store_ = new boost::thread(boost::bind(dataStore));

    ros::AsyncSpinner spinner(10); spinner.start();
    ROS_INFO("(handover) node started");

    //wrist sensor

    auto sub_h = node.subscribe(SENSOR_TOPIC, 1, &sensorReadCallbackWrist);
    auto sub_f = node.subscribe(SENSOR_TOPIC, 1, &sensorReadCallbackFingers);

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





    //arm control

    ROS_INFO("(handover) setting up control queue");
    cout <<  endl;
    auto robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", node));

    ROS_INFO("(handover) creating moveit kinematics instance");
    cout << endl;
    vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    auto mvKin = make_shared<MoveItKinematics>(robotinoQueue, node, "robotino", controlledJoints, "arm_link5");

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

    int grasp_value = 100;
    store_set_ = false;

    while(end_task > 0) {

        string numo, nums;

        cout << "(handover) enter subject id number: ";
        cin >> nums;

        cout << "(handover) enter object id number: ";
        cin >> numo;

        experiment_ = "subject" + nums + "object" + numo;

        store_ = true;
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

        //firstJoints = robotinoQueue->getCurrentJoints().joints;
        //cout << "(handover) current robot state: " << firstJoints.t() << endl;

        ROS_INFO("(handover) waiting to grasp the object");
        stage = 3;
        cout << "(handover) current stage "<<stage<<endl;


        cout << "(handover) press 1 to close the hand" << endl;
        cin >> grasp_value;
        stage = 4   ; //grasping the object
        cout << "(handover) current stage "<<stage<<endl;

        if(grasp_value == 1){
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

        //firstJoints = robotinoQueue->getCurrentJoints().joints;
        //cout << "(handover) current robot state: " << firstJoints.t() << endl;

        ROS_INFO("(handover) going to the handover pose with the closed hand");
        stage = 6; // initial pose with the closed hand
        cout << "(handover) current stage "<<stage<<endl;

        robotinoQueue->jointPtp(end);

        ROS_INFO("(handover) waiting to release the object");
        stage = 7;
        cout << "(handover) current stage "<<stage<<endl;


        cout << "(handover) press 1 to open the hand" << endl;
        cin >> grasp_value;
        stage = 8  ; //releasing the object
        cout << "(handover) current stage "<<stage<<endl;

        if(grasp_value == 1){
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

        store_ = false;
        while(!writing_done_){
            sleep(1);
        }
        cout << "(handover) press 1 to start handover / 0 to exit" << endl;
        cin >> end_task;


    }
    data_store_->join();

    getchar();

    return 0;

}

void sensorReadCallbackWrist(std_msgs::Float64MultiArray msg){

    wrist_sensor_values_.at(0) = msg.data.at(0);
    wrist_sensor_values_.at(1) = msg.data.at(1);
    wrist_sensor_values_.at(2) = msg.data.at(2);
    wrist_sensor_values_.at(3) = msg.data.at(3);
    wrist_sensor_values_.at(4) = msg.data.at(4);
    wrist_sensor_values_.at(5) = msg.data.at(5);

}

void sensorReadCallbackFingers(std_msgs::Float64MultiArray msg){

    fingertip_sensor_values_.at(0) = msg.data.at(0);
    fingertip_sensor_values_.at(1) = msg.data.at(1);
    fingertip_sensor_values_.at(2) = msg.data.at(2);
    fingertip_sensor_values_.at(3) = msg.data.at(3);
    fingertip_sensor_values_.at(4) = msg.data.at(4);
    fingertip_sensor_values_.at(5) = msg.data.at(5);
    fingertip_sensor_values_.at(6) = msg.data.at(6);
    fingertip_sensor_values_.at(7) = msg.data.at(7);
    fingertip_sensor_values_.at(8) = msg.data.at(8);
    fingertip_sensor_values_.at(9) = msg.data.at(9);
    fingertip_sensor_values_.at(10) = msg.data.at(10);
    fingertip_sensor_values_.at(11) = msg.data.at(11);
    fingertip_sensor_values_.at(12) = msg.data.at(12);
    fingertip_sensor_values_.at(13) = msg.data.at(13);
    fingertip_sensor_values_.at(14) = msg.data.at(14);
}


void dataStore(){
    ros::Rate lRate(10.0);
    auto start_time = ros::Time::now().toSec();
    tf::TransformListener tf_listener_;
    tf::StampedTransform trans;
    geometry_msgs::Pose pose_wrist_;
    vector<double> projected_sensor_values_;

    std::vector<double> TimeVector;
    std::vector<int> StageVector;
    std::vector<std::vector<double>>WristSensorValues;
    std::vector<std::vector<double>> FingertipSensorValues;
    std::vector<std::vector<double>> projectedSensorValues;
    std::vector<geometry_msgs::Pose> poseWristVector;

    std::ofstream rFile;
    string nameF;

    while(end_task>0){

        if(!store_set_ && store_){
            cout <<"(handovers) clear data "<<endl;
            store_set_ = true;
            writing_done_ = false;
            TimeVector.clear();
            TimeVector.shrink_to_fit();
           WristSensorValues.clear();
           WristSensorValues.shrink_to_fit();
            projectedSensorValues.clear();
            projectedSensorValues.shrink_to_fit();
            poseWristVector.clear();
            poseWristVector.shrink_to_fit();
            StageVector.clear();
            StageVector.shrink_to_fit();
            start_time = ros::Time::now().toSec();

        }
        if(store_set_ && store_){

            TimeVector.push_back(ros::Time::now().toSec() - start_time);
            tf_listener_.waitForTransform(base_frame_, wrist_frame_, ros::Time::now(), ros::Duration(0.1));
            tf_listener_.lookupTransform(base_frame_, wrist_frame_, ros::Time(0), trans);
            pose_wrist_ = tf_stamped2pose(trans);
            poseWristVector.push_back(pose_wrist_);
            sensor_mutex_.lock();
            WristSensorValues.push_back(wrist_sensor_values_);
            FingertipSensorValues.push_back(fingertip_sensor_values_);
            projected_sensor_values_ = projectReadings(wrist_sensor_values_, pose_wrist_);
            sensor_mutex_.unlock();
            projectedSensorValues.push_back(projected_sensor_values_);
            StageVector.push_back(stage);

        }

        if(store_set_ && !store_){
            cout<<"(handover) writing to a file"<<endl;
            store_set_ = false;
            nameF = path_  + experiment_ + ".txt";
            rFile.open(nameF.c_str());
            if(rFile.is_open()){
                cout << "File open"<<endl;
            }
            else {
                cout  << "File not open"<<endl;
            }
            rFile<< "time" << "\t" << "stage" << "\t" << "wrist.pos.x" << "\t" << "wrist.pos.y" << "\t" << "wrist.pos.z" << "\t" << "wrist.orient.x" << "\t" << "wrist.orient.y" << "\t" << "wrist.orient.z" << "\t" << "wrist.orient.w" << "\t" <<"force.x" << "\t" <<"force.y" << "\t" <<"force.z" << "\t" <<"torque.x" << "\t" <<"torque.y" <<"\t" <<"torque.z" << "\t"<<"projected.force.x" << "\t" <<"projected.force.y" << "\t" <<"projected.force.z" << "\t" <<"projected.torque.x" << "\t" <<"projected.torque.y" <<"\t" <<"projected.torque.z" << "\t";

            if(robot == tuw_robotino){
                rFile << "Fz1" << "\t" << " Mx1" << "\t" << " My1" << "\t" << " Fz2" << "\t" << " Mx2" << "\t" << " My2" << "\t" << " Fz3" << "\t" << " Mx3" << "\t" << " My3" << "\t" << " Dd1" << "\t" << " Dp1" << "\t" << " Dd2" << "\t" << " Dp2" << "\t" << " Dd3" << "\t" << " Dp3";
            }
            rFile << endl;
            for (int i = 0; i < TimeVector.size(); ++i){

                rFile << TimeVector.at(i)<< "\t";
                rFile << StageVector.at(i)<< "\t";
                rFile << poseWristVector.at(i).position.x << "\t";
                rFile << poseWristVector.at(i).position.y << "\t";
                rFile << poseWristVector.at(i).position.z << "\t";
                rFile << poseWristVector.at(i).orientation.x << "\t";
                rFile << poseWristVector.at(i).orientation.y << "\t";
                rFile << poseWristVector.at(i).orientation.z << "\t";
                rFile << poseWristVector.at(i).orientation.w << "\t";
                rFile <<WristSensorValues.at(i).at(0) << "\t";
                rFile <<WristSensorValues.at(i).at(1) << "\t";
                rFile <<WristSensorValues.at(i).at(2) << "\t";
                rFile <<WristSensorValues.at(i).at(3) << "\t";
                rFile <<WristSensorValues.at(i).at(4) << "\t";
                rFile <<WristSensorValues.at(i).at(5) << "\t";
                rFile << projectedSensorValues.at(i).at(0) << "\t";
                rFile << projectedSensorValues.at(i).at(1) << "\t";
                rFile << projectedSensorValues.at(i).at(2) << "\t";
                rFile << projectedSensorValues.at(i).at(3) << "\t";
                rFile << projectedSensorValues.at(i).at(4) << "\t";
                rFile << projectedSensorValues.at(i).at(5) << "\t";
                if(robot == tuw_robotino){
                    rFile <<FingertipSensorValues.at(i).at(0) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(1) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(2) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(3) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(4) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(5) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(6) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(7) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(8) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(9) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(10) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(11) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(12) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(13) << "\t";
                    rFile <<FingertipSensorValues.at(i).at(14) << "\t";

                }

                rFile << endl;

            }


            rFile.close();
            writing_done_ = true;
        }
        lRate.sleep();
    }

}

vector<double> projectVectors(double vecX,double vecY,double vecZ,double alpha,double beta,double gamma){
    vector<double> newVec;
    double i= vecX * (cos(beta)*cos(gamma)) + vecY *(cos(gamma)* sin(alpha)* sin(beta) - cos(alpha)*sin(gamma)) + vecZ *(cos(alpha)*cos(gamma)*sin(beta) +sin(alpha)*sin(gamma));
    double j= vecX * (cos(beta)*sin(gamma)) + vecY *(cos(alpha)* cos(gamma) + sin(alpha)*sin(gamma)*sin(beta)) + vecZ *(-1*cos(gamma)*sin(alpha) +cos(alpha)*sin(beta)*sin(gamma));
    double k= vecX * (-1*sin(beta)) + vecY *(cos(beta)* sin(alpha)) + vecZ *(cos(alpha)*cos(beta)); // ref: roll-x-alpha pitch-y-beta yaw-z-gamma
    newVec.push_back(i);
    newVec.push_back(j);
    newVec.push_back(k);
    return newVec;

}
std::vector<double> projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose){
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

geometry_msgs::Pose tf_stamped2pose(tf::StampedTransform tf_in){
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


