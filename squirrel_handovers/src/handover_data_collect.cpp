#include <vector>
#include <memory>
#include <iostream>
#include <cmath>
#include <boost/thread.hpp>

#include <geometry_msgs/Wrench.h>

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
std::vector<double>  robot_joints_;

bool write_file_set_(false), writing_(false);
int stage;
int end_task = 1;


string path_;
string experiment_ = "../../../data/";

void sensorReadCallback(std_msgs::Float64MultiArray msg);
void dataStore();
std::vector<geometry_msgs::Wrench> SensorValues;
std::vector<double> TimeVector;
std::vector<int> StageVector;
std::vector<std::vector<double>> robotJointsVector;
std::vector<geometry_msgs::Pose> endEffectorVector;


int main(int argc, char** args) {

    ros::init(argc, args, "handover_data_collection");
    ros::NodeHandle node;
    sleep(1);
    boost::thread* data_store_ = new boost::thread(boost::bind(dataStore));


    ros::AsyncSpinner spinner(10); spinner.start();
    //wrist sensor
    node.subscribe(SENSOR_TOPIC, 1, &sensorReadCallback);

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
        end_effector_ = mvKin->computeFk(armadilloToStdVec(robotinoQueue->getCurrentJoints().joints));
        cout << "end_effector "<< end_effector_<<endl;
        cout <<  "sensor value " << wrist_sensor_ <<endl;
        //auto projectedReadings=projectReadings(scaledReadings,mvKin->computeFk(armadilloToStdVec(robotinoQueue->getCurrentJoints().joints)));


//        ROS_INFO("(handover) going to the handover pose with the open hand");
//        stage = 2;
//        robotinoQueue->jointPtp(end);

//        firstJoints = robotinoQueue->getCurrentJoints().joints;
//        cout << "(handover) current robot state: " << firstJoints.t() << endl;
//        end_effector_ = mvKin->computeFk(armadilloToStdVec(robotinoQueue->getCurrentJoints().joints));

//        ROS_INFO("(handover) waiting to grasp the object");
//        stage = 3;

//        cout << "(handover) press 1 to close the hand" << endl;
//        cin >> grasp_value;
//        if(grasp_value == 1){
//            stage = 4; //grasping the object
//            cout<<"OK"<<endl;

//            if ( ros::service::call(HAND_SERVICE, graspService) ){
//                ROS_INFO("HAND Grasped!");
//            }else{
//                ROS_ERROR("FAILED to Graps!");
//            }
//        }

//        ROS_INFO("(handover) going to initial pose with the closed hand");
//        stage = 5;
//        robotinoQueue->jointPtp(start);

//        stage = 6; // initial pose with the closed hand
//        firstJoints = robotinoQueue->getCurrentJoints().joints;
//        cout << "(handover) current robot state: " << firstJoints.t() << endl;
//        end_effector_ = mvKin->computeFk(armadilloToStdVec(robotinoQueue->getCurrentJoints().joints));

//        ROS_INFO("(handover) going to the handover pose with the closed hand");
//        stage = 7;
//        robotinoQueue->jointPtp(end);
//        end_effector_ = mvKin->computeFk(armadilloToStdVec(robotinoQueue->getCurrentJoints().joints));


//        ROS_INFO("(handover) waiting to release the object");
//        stage = 8;

//        cout << "(handover) press 1 to open the hand" << endl;
//        cin >> grasp_value;
//        if(grasp_value == 1){
//            stage = 9   ; //releasing the object
//            cout<<"OK"<<endl;
//            if ( ros::service::call(HAND_SERVICE, releaseService) ){
//                ROS_INFO("HAND Released!");
//            }else{
//                ROS_ERROR("FAILED to Release!");
//            }
//        }

//        ROS_INFO("(handover) going to initial pose with the open hand");
//        stage = 0;
//        robotinoQueue->jointPtp(start);
//        end_effector_ = mvKin->computeFk(armadilloToStdVec(robotinoQueue->getCurrentJoints().joints));

        cout << "write " <<endl;
        writing_ = false;
        cout << "(handover) press 1 to start handover / 0 to exit" << endl;
        cin >> end_task;


    }
    data_store_->join();

    getchar();

    return 0;

}

void sensorReadCallback(std_msgs::Float64MultiArray msg){
    cout<<"callback msg "<<endl;

    wrist_sensor_.force.x=msg.data.at(0);
    wrist_sensor_.force.y=msg.data.at(1);
    wrist_sensor_.force.z=msg.data.at(2);

    wrist_sensor_.torque.x=msg.data.at(3);
    wrist_sensor_.torque.y=msg.data.at(4);
    wrist_sensor_.torque.z=msg.data.at(5);
}

void dataStore(){
    ros::Rate lRate(20.0);
    auto start_time = ros::Time::now().toSec();

    while(end_task>0){


        if(!write_file_set_ && writing_){
            cout <<"clear data "<<endl;
            write_file_set_ = true;
            TimeVector.clear();
            SensorValues.clear();
            StageVector.clear();
            endEffectorVector.clear();
            start_time = ros::Time::now().toSec();

        }
        if(write_file_set_&&writing_){
            TimeVector.push_back(ros::Time::now().toSec() - start_time);
            sensor_mutex_.lock();
            SensorValues.push_back(wrist_sensor_);
            endEffectorVector.push_back(end_effector_);
            sensor_mutex_.unlock();
            cout << "stage "<<stage<<endl;
            StageVector.push_back(stage);


        }

        if(write_file_set_ && !writing_){
            cout<<"writing to a file"<<endl;
            cout << "length end effecto vec "<<endEffectorVector.size() <<endl;
            write_file_set_ = false;
            std::ofstream rFile;
            string nameF = path_  + experiment_ + ".txt";
            rFile.open(nameF.c_str());
            rFile<< "time" << "\t" << "stage" << "\t" << "wrist.pos.x" << "\t" << "wrist.pos.y" << "\t" << "wrist.pos.z" << "\t" << "wrist.orient.x" << "\t" << "wrist.orient.y" << "\t" << "wrist.orient.z" << "\t" << "wrist.orient.w" << "\t" <<"force.x" << "\t" <<"force.y" << "\t" <<"force.z" << "\t" <<"torque.x" << "\t" <<"torque.y" <<"\t" <<"torque.z" << "\t"<< endl;
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


