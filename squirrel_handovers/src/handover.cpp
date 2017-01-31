#include <squirrel_handovers/handover.hpp>


using namespace std;
using namespace arma;
using namespace kukadu;

HandoverAction::HandoverAction(const std::string std_HandoverServerActionName) :
    handoverServer(nh, std_HandoverServerActionName, boost::bind(&HandoverAction::executeHandover, this, _1), false),
    private_nh("~")
{
    cout << "here 1"<<endl;
    private_nh.param("base_frame", base_frame_, std::string("/base_link"));
    private_nh.param("wrist_frame", wrist_frame_, std::string("/arm_link5"));
    private_nh.param("tuw_robotino", tuw_robotino, std::string("tuw-robotino2"));
    private_nh.param("uibk_robotino", uibk_robotino, std::string("uibk-robotino2-sh"));
    private_nh.param("robot", robot, std::string("uibk-robotino2-sh"));

    cout << "here 2"<<endl;

    sub_h = nh.subscribe(SENSOR_TOPIC, 1, &HandoverAction::sensorReadCallbackWrist,this);

    cout << "here 3"<<endl;
    if(robot == tuw_robotino){
        sub_f = nh.subscribe(FINGERTIP_TOPIC, 1, &HandoverAction::sensorReadCallbackFingers, this);
    }
    cout << "here 4"<<endl;

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

    auto firstJoints = robotinoQueue->getCurrentJoints().joints;
    cout << "(handover) current robot state: " << firstJoints.t() << endl;

    auto start = stdToArmadilloVec({0.0, 0.0, 0.0, 1.0, -0.5, 1.4, 1.0, 1.6});
    auto end = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});
    auto end_type1 = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});
    auto end_type2 = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});
    auto end_type3 = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});
    auto end_type4 = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});

    //robot shall not move base

    copy(firstJoints.begin(), firstJoints.begin() + 3, start.begin());
    copy(firstJoints.begin(), firstJoints.begin() + 3, end.begin());


    //handover type

    if (goal->handover_type == 1){
        copy(end_type1.begin() + 3, end_type1.end(), end.begin() + 3);
    }
    else if (goal->handover_type == 2){
        copy(end_type2.begin() + 3, end_type2.end(), end.begin() + 3);

    }
    else if (goal->handover_type == 3){
        copy(end_type3.begin() + 3, end_type3.end(), end.begin() + 3);

    }
    else if (goal->handover_type == 4){
        copy(end_type4.begin() + 3, end_type4.end(), end.begin() + 3);
    }

    cout<<"(handover)current target handover"<<endl<<end<<endl;


    bool handover_success_ = false;
    stage = 0;
    
    std::string take ("take");
    std::string give ("give");

    if(take.compare(goal->action_type.c_str())==0){
        sleep(1);

        //make sure hand is open

        if (robot == uibk_robotino){

            if ( ros::service::call(HAND_SERVICE, releaseService) ){
                ROS_INFO("(handover) HAND Released!");
                handover_success_ = true;
            }else{
                ROS_ERROR("handover) FAILED to Release!");
                handover_success_ = false;
            }
        }
        else if (robot == tuw_robotino){
            kclhandGraspActionClient.sendGoal(releaseServiceKCL.goal);
            kclhandGraspActionClient.waitForResult(ros::Duration(5.0));
            if (kclhandGraspActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("(handover) HAND Released!");
                handover_success_ = true;
            }else{
                ROS_ERROR("handover) FAILED to Release!");
                handover_success_ = false;
            }
        }

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

        torque_past.clear();
        force_past.clear();

        if (record_magnitude(current_forces_, current_torques_))
        {
            grasp_value = detector();
        }


        if(grasp_value){

            // stage = 4; //grasping the object
            cout<<"OK"<<endl;

            if (robot == uibk_robotino){

                if ( ros::service::call(HAND_SERVICE, graspService) ){
                    ROS_INFO("(handover) HAND Grasped!");
                    handover_success_ = true;
                }else{
                    ROS_ERROR("handover) FAILED to Graps!");
                    handover_success_ = false;
                }
            }
            else if (robot == tuw_robotino){
                kclhandGraspActionClient.sendGoal(graspServiceKCL.goal);
                kclhandGraspActionClient.waitForResult(ros::Duration(5.0));
                if (kclhandGraspActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("(handover) HAND Grasped!");
                    handover_success_ = true;
                }else{
                    ROS_ERROR("handover) FAILED to Graps!");
                    handover_success_ = false;
                }
            }
        }
        cout << "(handover) current stage "<<stage<<endl;

        ROS_INFO("(handover) going to initial pose with the closed hand");
        stage = 5;
        cout << "(handover) current stage "<<stage<<endl;
        robotinoQueue->jointPtp(start);
    }
    else if(give.compare(goal->action_type.c_str()) ==0){

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
                    handover_success_ = true;
                }else{
                    ROS_ERROR("handover) FAILED to Release!");
                    handover_success_ = false;
                }
            }
            else if (robot == tuw_robotino){
                kclhandGraspActionClient.sendGoal(releaseServiceKCL.goal);
                kclhandGraspActionClient.waitForResult(ros::Duration(5.0));
                if (kclhandGraspActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("(handover) HAND Released!");
                    handover_success_ = true;
                }else{
                    ROS_ERROR("handover) FAILED to Release!");
                    handover_success_ = false;
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
        handover_success_ = false;
    }

    ROS_INFO(" Handover: Handover sequence finished.");
    cout<<endl;
    


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
    copy( wrist_sensor_values_.begin(), wrist_sensor_values_.begin() + 3, current_forces_.begin());
    copy( wrist_sensor_values_.begin() + 3, wrist_sensor_values_.end(), current_torques_.begin());

}

void HandoverAction::sensorReadCallbackFingers(std_msgs::Float64MultiArray msg){

    fingertip_sensor_values_ = msg.data;
}

bool HandoverAction::record_magnitude(const std::vector<double>& frc, const std::vector<double>& trq)
{
    double frc_mag = sqrt(pow(frc[X], 2) + pow(frc[Y], 2) + pow(frc[Z], 2));
    double trq_mag = sqrt(pow(trq[X], 2) + pow(trq[Y], 2) + pow(trq[Z], 2));

    int f_size = force_past.size();
    int t_size = torque_past.size();
    if (f_size < MIN_VALS)
    {
        force_past.push_back(frc_mag);
        ++f_size;
    }
    else
    {
        double tmp=force_past[MIN_VALS-1];
        force_past[MIN_VALS - 1] = frc_mag;			//last value
        for (int i = MIN_VALS - 2; i >= 0; --i)	//value before the last
        {
            std::swap(force_past[i],tmp);
        }
    }

    if (t_size < MIN_VALS)
    {
        torque_past.push_back(trq_mag);
        ++t_size;
    }
    else
    {
        double tmp = torque_past[MIN_VALS - 1];
        torque_past[MIN_VALS - 1] = trq_mag;			//last value
        for (int i = MIN_VALS - 2; i >= 0; --i)	//value before the last
        {
            std::swap(torque_past[i], tmp);
        }
    }

    if (t_size == MIN_VALS && f_size == MIN_VALS)	//if we acquired the right amount of data
    {
        return true;
    }
    return false;	//too few or too many values
}

bool HandoverAction::detector()
{
    //if we don't get what expected, fail
    assert(force_past.size() == MIN_VALS && torque_past.size() == MIN_VALS);

    std::vector<double> f_diffs;
    std::vector<double> t_diffs;

    int idx=force_past.size();

    //this can be optimised in a for loop if needed
    //NOTE here the order is reversed: v[0] is the newest - if the for loop is increasing, we will have the same order
    f_diffs.push_back(force_past.at(idx-1) - force_past.at(idx - 2));
    f_diffs.push_back(force_past.at(idx-2) - force_past.at(idx - 3));
    t_diffs.push_back(torque_past.at(idx-1) - torque_past.at(idx - 2));
    t_diffs.push_back(torque_past.at(idx-2) - torque_past.at(idx - 3));

    //normally we should always have exactly 2 values, if we don't something's gone wrong
    assert(f_diffs.size() == 2 && t_diffs.size() == 2);

    //newest - oldest diffs, true if the diff of the diff is either bigger than 1 or less than -1
    bool f_good = ((f_diffs.at(1) - f_diffs.at(0)) > 1 || (f_diffs.at(1) - f_diffs.at(0)) < -1) ? true : false;
    bool t_good = ((t_diffs.at(1) - t_diffs.at(0)) > 1 || (t_diffs.at(1) - t_diffs.at(0)) < -1) ? true : false;

    return f_good;	//return true only if force threashold is good, torque is for future use
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "handover");
    sleep(1); ros::AsyncSpinner spinner(10); spinner.start();

    HandoverAction hadnover(HANDOVER_NAME);

    getchar();
    return 0;

}


