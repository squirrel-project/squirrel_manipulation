#include <squirrel_object_manipulation/RobotinoControl.hpp>
#include <squirrel_manipulation_msgs/PickUpAction.h>
#include <squirrel_manipulation_msgs/BlindGraspAction.h>
#include <squirrel_manipulation_msgs/DropAction.h>
#include <squirrel_manipulation_msgs/PutDownAction.h>
#include <squirrel_manipulation_msgs/InspectAction.h>
#include <squirrel_manipulation_msgs/LeanAction.h>
#include <actionlib/server/simple_action_server.h>

#define SQRL_PICKUP_NAME "pickup"
#define SQRL_BLIND_GRASP_NAME "blindgrasp"
#define SQRL_DROP_NAME "drop"
#define SQRL_PUT_DOWN_NAME "putdown"
#define SQRL_INSPECT_NAME "inspect"
#define SQRL_LEAN_NAME "lean"

class ManipulationAction {
protected:

    ros::NodeHandle nh;

    actionlib::SimpleActionServer<squirrel_manipulation_msgs::PickUpAction> pickServer;
    actionlib::SimpleActionServer<squirrel_manipulation_msgs::BlindGraspAction> blindServer;
    actionlib::SimpleActionServer<squirrel_manipulation_msgs::DropAction> dropServer;
    actionlib::SimpleActionServer<squirrel_manipulation_msgs::PutDownAction> putServer;
    actionlib::SimpleActionServer<squirrel_manipulation_msgs::InspectAction> inspectServer;
    actionlib::SimpleActionServer<squirrel_manipulation_msgs::LeanAction> leanServer;

    squirrel_manipulation_msgs::PickUpFeedback pickupFeedback;
    squirrel_manipulation_msgs::BlindGraspFeedback blindFeedback;
    squirrel_manipulation_msgs::DropFeedback dropFeedback;
    squirrel_manipulation_msgs::PutDownFeedback putFeedback;
    squirrel_manipulation_msgs::InspectFeedback inspectFeedback;
    squirrel_manipulation_msgs::LeanFeedback leanFeedback;

    squirrel_manipulation_msgs::PickUpResult pickupResult;
    squirrel_manipulation_msgs::BlindGraspResult blindResult;
    squirrel_manipulation_msgs::DropResult dropResult;
    squirrel_manipulation_msgs::PutDownResult putResult;
    squirrel_manipulation_msgs::InspectResult inspectResult;
    squirrel_manipulation_msgs::LeanResult leanResult;

public:

    ManipulationAction(const std::string pickupServerActionName, const std::string blindServerActionName, const std::string dropServerActionName, const std::string putServerActionName, const std::string inspectServerActionName, const std::string leanServerActionName);
    ~ManipulationAction();
    
    void executePickUp(const squirrel_manipulation_msgs::PickUpGoalConstPtr &goal);
    void executeBlindGrasp(const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal);
    void executeDrop(const squirrel_manipulation_msgs::DropGoalConstPtr &goal);
    void executePut(const squirrel_manipulation_msgs::PutDownGoalConstPtr &goal);
    void executeInspect(const squirrel_manipulation_msgs::InspectGoalConstPtr &goal);
    void executeLean(const squirrel_manipulation_msgs::LeanGoalConstPtr &goal);

};
