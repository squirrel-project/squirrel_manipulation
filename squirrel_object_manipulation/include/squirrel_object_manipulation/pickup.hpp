#include <squirrel_object_manipulation/RobotinoControl.hpp>
#include <squirrel_manipulation_msgs/PickUpAction.h>
#include <squirrel_manipulation_msgs/BlindGraspAction.h>
#include <actionlib/server/simple_action_server.h>

#define PICKUP_NAME "pickup"
#define BLIND_GRASP_NAME "blindgrasp"

class PickupAction {
protected:

    ros::NodeHandle nh;

    actionlib::SimpleActionServer<squirrel_manipulation_msgs::PickUpAction> pickServer;
    actionlib::SimpleActionServer<squirrel_manipulation_msgs::BlindGraspAction> blindServer;

    squirrel_manipulation_msgs::PickUpFeedback pickupFeedback;
    squirrel_manipulation_msgs::BlindGraspFeedback blindFeedback;

    squirrel_manipulation_msgs::PickUpResult pickupResult;
    squirrel_manipulation_msgs::BlindGraspResult blindResult;

public:

    PickupAction(const std::string pickupServerActionName, const std::string blindServerActionName);
    ~PickupAction();
    
    void executePickUp(const squirrel_manipulation_msgs::PickUpGoalConstPtr &goal);
    void executeBlindGrasp(const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal);


};
