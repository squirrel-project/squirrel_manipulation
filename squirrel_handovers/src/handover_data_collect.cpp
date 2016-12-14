#include <vector>
#include <memory>
#include <iostream>
#include <cmath>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <kukadu/kukadu.hpp>
#include <squirrel_manipulation_msgs/SoftHandGrasp.h>

#define SENSOR_TOPIC "/wrist"
#define HAND_SERVICE "/softhand_grasp"

static auto constexpr  SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR = -2.27;


using namespace std;
using namespace arma;
using namespace kukadu;

void sensorReadCallback(std_msgs::Float64MultiArray msg);
void dataStore();
geometry_msgs::Pose tf_stamped2pose(tf::StampedTransform tf_in);


string base_frame_ = "/base_link";
string wrist_frame_ = "/arm_link5";
string path_= "../../data/";
//string path_ ="/home/c7031098/squirrel_ws_new/data/";
string experiment_;

bool store_set_(false), store_(false);
int stage = 100;
int end_task = 100;

boost::mutex sensor_mutex_;
std::vector<double>  robot_joints_, wrist_sensor_values_;
std::vector<double> projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose);
std::vector<double> TimeVector;
std::vector<int> StageVector;
std::vector<std::vector<double>> SensorValues;
std::vector<std::vector<double>> projectedSensorValues;
std::vector<geometry_msgs::Pose> poseWristVector;



int main(int argc, char** args) {

    ros::init(argc, args, "handover_data_collection");
    ros::NodeHandle node;
    sleep(1);
    boost::thread* data_store_ = new boost::thread(boost::bind(dataStore));

    ros::AsyncSpinner spinner(10); spinner.start();
    //wrist sensor
    auto sub = node.subscribe(SENSOR_TOPIC, 1, &sensorReadCallback);

    //hand
    squirrel_manipulation_msgs::SoftHandGrasp graspService;
    graspService.request.position = 1.0;
    squirrel_manipulation_msgs::SoftHandGrasp releaseService;
    releaseService.request.position = 0.0;


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

    cout << "switching to impedance mode if it is not there yet" << endl;
    if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    }

    auto start = stdToArmadilloVec({0.0, 0.0, 0.0, 1.0, -0.5, 1.4, 1.0, 1.6});
    auto end = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});

    int grasp_value = 100;
    stage = 100;

    std::vector<double> joint_val;

    store_set_ = false;
    while(end_task > 0) {

        cout << "(handover) enter experiment name: ";
        cin >> experiment_;

        store_ = true;
        auto firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;

        ROS_INFO("(handover) going to initial pose with the open hand");
        stage = 0;
        cout << "(handover) current stage "<<stage<<endl;

        robotinoQueue->jointPtp(start);


        stage = 1; // initial pose with the open hand
        cout << "(handover) current stage "<<stage<<endl;
        firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;


        ROS_INFO("(handover) going to the handover pose with the open hand");
        stage = 2;
        cout << "(handover) current stage "<<stage<<endl;

        robotinoQueue->jointPtp(end);

        firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;

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

            if ( ros::service::call(HAND_SERVICE, graspService) ){
                ROS_INFO("(handover) HAND Grasped!");
            }else{
                ROS_ERROR("handover) FAILED to Graps!");
            }
        }
        cout << "(handover) current stage "<<stage<<endl;

        ROS_INFO("(handover) going to initial pose with the closed hand");
        stage = 5;
        cout << "(handover) current stage "<<stage<<endl;

        robotinoQueue->jointPtp(start);

        firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;

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
            if ( ros::service::call(HAND_SERVICE, releaseService) ){
                ROS_INFO("(handover) HAND Released!");
            }else{
                ROS_ERROR("handover) FAILED to Release!");
            }
        }


        stage = 9;
        cout << "(handover) current stage "<<stage<<endl;

        ROS_INFO("(handover) going to initial pose with the open hand");
        robotinoQueue->jointPtp(start);

        store_ = false;
        cout << "(handover) press 1 to start handover / 0 to exit" << endl;
        cin >> end_task;


    }
    data_store_->join();

    getchar();

    return 0;

}

void sensorReadCallback(std_msgs::Float64MultiArray msg){

    wrist_sensor_values_.clear();
    wrist_sensor_values_.push_back(msg.data.at(0));
    wrist_sensor_values_.push_back(msg.data.at(1));
    wrist_sensor_values_.push_back(msg.data.at(2));
    wrist_sensor_values_.push_back(msg.data.at(3));
    wrist_sensor_values_.push_back(msg.data.at(4));
    wrist_sensor_values_.push_back(msg.data.at(5));

}

void dataStore(){
    ros::Rate lRate(10.0);
    auto start_time = ros::Time::now().toSec();
    tf::TransformListener tf_listener_;
    tf::StampedTransform trans;
    geometry_msgs::Pose pose_wrist_;
    vector<double> projected_sensor_values_;

    while(end_task>0){
        // cout <<"in loop "<<endl;


        if(!store_set_ && store_){
            cout <<"clear data "<<endl;
            store_set_ = true;
            TimeVector.clear();
            SensorValues.clear();
            projectedSensorValues.clear();
            poseWristVector.clear();
            StageVector.clear();
            start_time = ros::Time::now().toSec();

        }
        if(store_set_ && store_){

            TimeVector.push_back(ros::Time::now().toSec() - start_time);
            tf_listener_.waitForTransform(base_frame_, wrist_frame_, ros::Time::now(), ros::Duration(0.1));
            tf_listener_.lookupTransform(base_frame_, wrist_frame_, ros::Time(0), trans);
            pose_wrist_ = tf_stamped2pose(trans);
            poseWristVector.push_back(pose_wrist_);
            sensor_mutex_.lock();
            SensorValues.push_back(wrist_sensor_values_);
            projected_sensor_values_ = projectReadings(wrist_sensor_values_, pose_wrist_);
            sensor_mutex_.unlock();
            projectedSensorValues.push_back(projected_sensor_values_);
            StageVector.push_back(stage);


        }

        if(store_set_ && !store_){
            cout<<"writing to a file"<<endl;
            store_set_ = false;
            std::ofstream rFile;
            string nameF = path_  + experiment_ + ".txt";
            rFile.open(nameF.c_str());
            rFile<< "time" << "\t" << "stage" << "\t" << "wrist.pos.x" << "\t" << "wrist.pos.y" << "\t" << "wrist.pos.z" << "\t" << "wrist.orient.x" << "\t" << "wrist.orient.y" << "\t" << "wrist.orient.z" << "\t" << "wrist.orient.w" << "\t" <<"projected.force.x" << "\t" <<"projected.force.y" << "\t" <<"projected.force.z" << "\t" <<"projected.torque.x" << "\t" <<"projected.torque.y" <<"\t" <<"projected.torque.z" << "\t"<<  endl;
            for (int i; i < TimeVector.size(); ++i){
                rFile << TimeVector.at(i)<< "\t";
                rFile << StageVector.at(i)<< "\t";
                rFile << poseWristVector.at(i).position.x << "\t";
                rFile << poseWristVector.at(i).position.y << "\t";
                rFile << poseWristVector.at(i).position.z << "\t";
                rFile << poseWristVector.at(i).orientation.x << "\t";
                rFile << poseWristVector.at(i).orientation.y << "\t";
                rFile << poseWristVector.at(i).orientation.z << "\t";
                rFile << poseWristVector.at(i).orientation.w << "\t";
                rFile << SensorValues.at(i).at(0) << "\t";
                rFile << SensorValues.at(i).at(1) << "\t";
                rFile << SensorValues.at(i).at(2) << "\t";
                rFile << SensorValues.at(i).at(3) << "\t";
                rFile << SensorValues.at(i).at(4) << "\t";
                rFile << SensorValues.at(i).at(5) << "\t";
                rFile << projectedSensorValues.at(i).at(0) << "\t";
                rFile << projectedSensorValues.at(i).at(1) << "\t";
                rFile << projectedSensorValues.at(i).at(2) << "\t";
                rFile << projectedSensorValues.at(i).at(3) << "\t";
                rFile << projectedSensorValues.at(i).at(4) << "\t";
                rFile << projectedSensorValues.at(i).at(5) << "\t";
                rFile <<endl;

            }


            rFile.close();
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


