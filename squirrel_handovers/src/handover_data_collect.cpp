#include <vector>
#include <memory>
#include <iostream>
#include <cmath>
#include <boost/thread.hpp>

#include <geometry_msgs/Wrench.h>
#include <tf/transform_listener.h>

#include <kukadu/kukadu.hpp>
#include <squirrel_manipulation_msgs/SoftHandGrasp.h>

#define SENSOR_TOPIC "/wrist"
#define HAND_SERVICE "/softhand_grasp"

static auto constexpr  SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR = -2.27;


using namespace std;
using namespace arma;
using namespace kukadu;

boost::mutex sensor_mutex_;
geometry_msgs::Wrench wrist_sensor_;
geometry_msgs::Pose end_effector_;
std::vector<double>  robot_joints_, wrist_sensor_values_;

bool write_file_set_(false), writing_(false);
int stage;
int end_task = 1;

string base_frame_ = "/base_link";
string wrist_frame_ = "/arm_link5";
string path_= "../../data/";
//string path_ ="/home/c7031098/squirrel_ws_new/data/";
string experiment_;

void sensorReadCallback(std_msgs::Float64MultiArray msg);
void dataStore();
std::vector<double> projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose);

geometry_msgs::Pose tf_stamped2pose(tf::StampedTransform tf_in);

std::vector<geometry_msgs::Wrench> SensorValues;
std::vector<double> TimeVector;
std::vector<int> StageVector;
std::vector<std::vector<double>> robotJointsVector;
std::vector<std::vector<double>> projectedSensorValues;
std::vector<geometry_msgs::Pose> endEffectorVector;
std::vector<geometry_msgs::Pose> poseWristVector;


int main(int argc, char** args) {

    ros::init(argc, args, "handover_data_collection");
    ros::NodeHandle node;
    sleep(1);
    boost::thread* data_store_ = new boost::thread(boost::bind(dataStore));
    boost::thread* poses_update_ = new boost::thread(boost::bind(dataStore));


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

    write_file_set_ = false;
    while(end_task > 0) {

        cout << "(handover) enter experiment name: ";
        cin >> experiment_;

        writing_ = true;
        auto firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;

        ROS_INFO("(handover) going to initial pose with the open hand");
        stage = 0;
        robotinoQueue->jointPtp(start);


        stage = 1; // initial pose with the open hand
        firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;
        joint_val = armadilloToStdVec(robotinoQueue->getCurrentJoints().joints);
        joint_val.at(0) = 0.0; joint_val.at(1) = 0.0; joint_val.at(2) = 0.0;
        end_effector_ = mvKin->computeFk(joint_val);
        //cout << "end_effector "<< end_effector_<<endl;
        //cout <<  "sensor value " << wrist_sensor_ <<endl;
        //auto projectedReadings=projectReadings(scaledReadings,mvKin->computeFk(armadilloToStdVec(robotinoQueue->getCurrentJoints().joints)));


        ROS_INFO("(handover) going to the handover pose with the open hand");
        stage = 2;
        robotinoQueue->jointPtp(end);

        firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;
        joint_val = armadilloToStdVec(robotinoQueue->getCurrentJoints().joints);
        joint_val.at(0) = 0.0; joint_val.at(1) = 0.0; joint_val.at(2) = 0.0;
        end_effector_ = mvKin->computeFk(joint_val);

        ROS_INFO("(handover) waiting to grasp the object");
        stage = 3;

        cout << "(handover) press 1 to close the hand" << endl;
        cin >> grasp_value;
        stage = 4   ; //grasping the object
        if(grasp_value == 1){
           // stage = 4; //grasping the object
            cout<<"OK"<<endl;

            if ( ros::service::call(HAND_SERVICE, graspService) ){
                ROS_INFO("HAND Grasped!");
            }else{
                ROS_ERROR("FAILED to Graps!");
            }
        }
        cout << "stage "<<stage<<endl;

        ROS_INFO("(handover) going to initial pose with the closed hand");
        stage = 5;
        robotinoQueue->jointPtp(start);

       // stage = 6; // initial pose with the closed hand
        firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;
        joint_val = armadilloToStdVec(robotinoQueue->getCurrentJoints().joints);
        joint_val.at(0) = 0.0; joint_val.at(1) = 0.0; joint_val.at(2) = 0.0;
        end_effector_ = mvKin->computeFk(joint_val);

        ROS_INFO("(handover) going to the handover pose with the closed hand");
        stage = 6;
        robotinoQueue->jointPtp(end);

        ROS_INFO("(handover) waiting to release the object");
        stage = 7;
        joint_val = armadilloToStdVec(robotinoQueue->getCurrentJoints().joints);
        joint_val.at(0) = 0.0; joint_val.at(1) = 0.0; joint_val.at(2) = 0.0;
        end_effector_ = mvKin->computeFk(joint_val);

        cout << "(handover) press 1 to open the hand" << endl;
        cin >> grasp_value;
        stage = 8  ; //releasing the object
        if(grasp_value == 1){
            //stage = 9   ; //releasing the object
            cout<<"OK"<<endl;
            if ( ros::service::call(HAND_SERVICE, releaseService) ){
                ROS_INFO("HAND Released!");
            }else{
                ROS_ERROR("FAILED to Release!");
            }
        }
        cout << "stage "<<stage<<endl;


        stage = 9;
        ROS_INFO("(handover) going to initial pose with the open hand");
        robotinoQueue->jointPtp(start);
        joint_val = armadilloToStdVec(robotinoQueue->getCurrentJoints().joints);
        joint_val.at(0) = 0.0; joint_val.at(1) = 0.0; joint_val.at(2) = 0.0;
        end_effector_ = mvKin->computeFk(joint_val);

        //cout << "write " <<endl;
        writing_ = false;
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


    wrist_sensor_.force.x=msg.data.at(0);
    wrist_sensor_.force.y=msg.data.at(1);
    wrist_sensor_.force.z=msg.data.at(2);

    wrist_sensor_.torque.x=msg.data.at(3);
    wrist_sensor_.torque.y=msg.data.at(4);
    wrist_sensor_.torque.z=msg.data.at(5);
}

void dataStore(){
    ros::Rate lRate(10.0);
    auto start_time = ros::Time::now().toSec();
    tf::TransformListener tf_listener_;
    tf::StampedTransform trans;
    geometry_msgs::Pose pose_wrist_;
    vector<double> projected_sensor_values_;

    while(end_task>0){
        //cout <<"in loop "<<endl;


        if(!write_file_set_ && writing_){
            cout <<"clear data "<<endl;
            write_file_set_ = true;
            TimeVector.clear();
            SensorValues.clear();
            poseWristVector.clear();
            StageVector.clear();
            endEffectorVector.clear();
            projectedSensorValues.clear();
            start_time = ros::Time::now().toSec();

        }
        if(write_file_set_&&writing_){
            TimeVector.push_back(ros::Time::now().toSec() - start_time);
            sensor_mutex_.lock();
            SensorValues.push_back(wrist_sensor_);

            tf_listener_.waitForTransform(base_frame_, wrist_frame_, ros::Time::now(), ros::Duration(0.1));
            tf_listener_.lookupTransform(base_frame_, wrist_frame_, ros::Time(0), trans);
            pose_wrist_ = tf_stamped2pose(trans);
            poseWristVector.push_back(pose_wrist_);

            projected_sensor_values_ = projectReadings(wrist_sensor_values_, pose_wrist_);
            projectedSensorValues.push_back(wrist_sensor_values_);

            endEffectorVector.push_back(end_effector_);
            sensor_mutex_.unlock();
            //cout << "stage "<<stage<<endl;
            StageVector.push_back(stage);


        }

        if(write_file_set_ && !writing_){
            cout<<"writing to a file"<<endl;
            //cout << "length end effector vec "<<endEffectorVector.size() <<endl;
            write_file_set_ = false;
            std::ofstream rFile;
            string nameF = path_  + experiment_ + ".txt";
            rFile.open(nameF.c_str());
            rFile<< "time" << "\t" << "stage" << "\t" << "wrist.pos.x" << "\t" << "wrist.pos.y" << "\t" << "wrist.pos.z" << "\t" << "wrist.orient.x" << "\t" << "wrist.orient.y" << "\t" << "wrist.orient.z" << "\t" << "wrist.orient.w" << "\t" <<"projected.force.x" << "\t" <<"projected.force.y" << "\t" <<"projected.force.z" << "\t" <<"projected.torque.x" << "\t" <<"projected.torque.y" <<"\t" <<"projected.torque.z" << "\t"<<  endl;
            for (int i; i < TimeVector.size(); ++i){
                rFile << TimeVector.at(i)<< "\t";
                rFile << StageVector.at(i)<< "\t";
                rFile << endEffectorVector.at(i).position.x << "\t";
                rFile << endEffectorVector.at(i).position.y << "\t";
                rFile << endEffectorVector.at(i).position.z << "\t";
                rFile << endEffectorVector.at(i).orientation.x << "\t";
                rFile << endEffectorVector.at(i).orientation.y << "\t";
                rFile << endEffectorVector.at(i).orientation.z << "\t";
                rFile << endEffectorVector.at(i).orientation.w << "\t";
                rFile << SensorValues.at(i).force.x << "\t";
                rFile << SensorValues.at(i).force.y << "\t";
                rFile << SensorValues.at(i).force.z << "\t";
                rFile << SensorValues.at(i).torque.x << "\t";
                rFile << SensorValues.at(i).torque.y << "\t";
                rFile << SensorValues.at(i).torque.z << "\t";
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
std::vector<double> projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose){ //memory problem?
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


