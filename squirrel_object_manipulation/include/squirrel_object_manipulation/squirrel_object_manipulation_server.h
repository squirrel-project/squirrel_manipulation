#ifndef SQUIRREL_OBJECT_MANIPULATION_SERVER_H_
#define SQUIRREL_OBJECT_MANIPULATION_SERVER_H_

#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <squirrel_manipulation_msgs/BlindGraspAction.h>
#include <squirrel_motion_planner_msgs/PlanEndEffector.h>
#include <squirrel_motion_planner_msgs/PlanPose.h>
#include <squirrel_motion_planner_msgs/SendControlCommand.h>
#include <squirrel_motion_planner_msgs/UnfoldArm.h>
#include <kclhand_control/HandOperationMode.h>
#include <squirrel_manipulation_msgs/SoftHandGrasp.h>

#define NODE_NAME_ "squirrel_object_manipulation_server"
#define METAHAND_STRING_ "metahand"
#define SOFTHAND_STRING_ "softhand"
#define MAP_FRAME_ "map"
#define PLANNING_FRAME_ "map"
#define NUM_BASE_JOINTS_ 3
#define NUM_ARM_JOINTS_ 5
#define PLANNING_TIME_ 3.0
#define PLAN_WITH_OCTOMAP_COLLISIONS_ true
#define PLAN_WITH_SELF_COLLISIONS_ true
#define MAX_WAIT_TRAJECTORY_COMPLETION_ 60.0  // seconds
#define JOINT_IN_POSITION_THRESHOLD_ 0.139626 // 8 degrees
#define APPROACH_HEIGHT_ 0.15  // cm

// See KCL hand control
#define CLOSE_METAHAND_OPERATION_MODE 2
#define OPEN_METAHAND_OPERATION_MODE 3

// See softhand
#define CLOSE_SOFTHAND_VALUE 0.0
#define OPEN_SOFTHAND_VALUE 0.9

/**
 * \brief Object Manipulation server to perform grasping, placing, hand opening and closing
 *        Operates with the KCL metahand and the UIBK softhand, this is a parameter given to the ros node
 *        Depends on the squirrel_motion_planner for planning and sending trajectories to the arm
 * \author Tim Patten (patten@acin.tuwien.ac.at)
 */
class SquirrelObjectManipulationServer
{
    public:

    /**
    * \brief The HandType enum that stores which type of hand to operate with
    */
    enum HandType
    {
        UNKNOWN_HAND = 0,
        METAHAND = 1,
        SOFTHAND = 2
    };

    /**
    * \brief The ActionType enum that specifies which action has been called by the action client
    */
    enum ActionType
    {
        UNKNOWN_ACTION = 0,
        OPEN_HAND_ACTION = 1,
        CLOSE_HAND_ACTION = 2,
        GRASP = 3,
        PLACE = 4
    };

    /**
    * \brief The HandActuation enum that specifies which hand action to apply
    */
    enum HandActuation
    {
        OPEN = 0,
        CLOSE = 1
    };

    /**
    * \brief Constructor with ros node handle and action name
    * \param[in] n The ros node handle
    * \param[in] action_name The name to give the action server
    */
    SquirrelObjectManipulationServer ( ros::NodeHandle &n, const std::string &action_name );

    /**
    * \brief Destructor
    */
    virtual ~SquirrelObjectManipulationServer ();

    /**
    * \brief Initialize the server with hand type
    * \param[in] hand_name The name of the hand to operate with
    * \returns True if hand_type is reconized and if trajectory_folding_arm is found on the ros parameter server
    */
    bool initialize ( const std::string &hand_name );

    private:

    // Node handle
    ros::NodeHandle *n_;
    // Action server
    actionlib::SimpleActionServer<squirrel_manipulation_msgs::BlindGraspAction> as_;
    // Action name
    std::string action_name_;
    // Hand type
    HandType hand_type_;
    // Action type
    ActionType action_type_;
    // Hand available (only available with real robot)
    bool hand_available_;
    // Messages to publish feedback and result
    squirrel_manipulation_msgs::BlindGraspFeedback feedback_;
    squirrel_manipulation_msgs::BlindGraspResult result_;
    // Services
    ros::ServiceClient *arm_unfold_client_;
    ros::ServiceClient *arm_end_eff_planner_client_;
    ros::ServiceClient *arm_pose_planner_client_;
    ros::ServiceClient *arm_send_trajectory_client_;
    ros::ServiceClient *hand_client_;
    // Messages
    squirrel_motion_planner_msgs::UnfoldArm unfold_goal_;
    squirrel_motion_planner_msgs::PlanEndEffector end_eff_goal_;
    squirrel_motion_planner_msgs::PlanPose pose_goal_;
    squirrel_motion_planner_msgs::SendControlCommand cmd_goal_;
    kclhand_control::HandOperationMode metahand_goal_;
    squirrel_manipulation_msgs::SoftHandGrasp softhand_goal_;
    // Joint callback
    ros::Subscriber joints_state_sub_;
    std::vector<double> current_joints_;
    // Joints command callback
    ros::Subscriber joints_command_sub_;
    std::vector<double> current_cmd_;
    // Marker publisher
    ros::Publisher goal_pose_pub_;
    visualization_msgs::Marker goal_marker_;
    // Transform
    tf::TransformListener tf_listener_;
    // Important poses
    std::vector<double> folded_pose;
    std::vector<double> unfolded_pose;

    /**
     * \brief The main action server callback function
     * \param[in] goal A pointer to BlindGrasp action goal
     */
    void actionServerCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

    /**
     * \brief Dispatches the hand actuation call to either metahand or softhand actuation
     * \param[in] hand_actuation The type of hand actuation (open or close)
     * \returns True if hand successfully actuates
     */
    bool actuateHand ( const HandActuation &hand_actuation );

    /**
     * \brief Actuates the metahand
     * \param[in] hand_actuation The type of hand actuation (open or close)
     * \returns True if metahand successfully actuates
     */
    bool actuateMetahand ( const HandActuation &hand_actuation );

    /**
     * \brief Actuates the softhand
     * \param[in] hand_actuation The type of hand actuation (open or close)
     * \returns True if softhand successfully actuates
     */
    bool actuateSofthand ( const HandActuation &hand_actuation );

    /**
     * \brief Series of actions to grasp an object given a goal grasp pose
     *        Actions are: open hand, move to approach pose, move to grasp pose, close hand, retract arm
     * \param[in] goal The goal grasp pose
     * \returns True if all actions are performed successfully
     */
    bool grasp ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

    /**
     * \brief Series of actions to place an object given a goal place pose
     *        Actions are: move to approach pose, move to place pose, open hand, retract arm
     * \param[in] goal The goal place pose
     * \returns True if all actions are performed successfully
     */
    bool place ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal);

    /**
     * \brief Moves the end effector to a 6DOF pose in the map frame
     * \param[in] x The x coordinate of pose
     * \param[in] y The y coordinate of pose
     * \param[in] z The z coordinate of pose
     * \param[in] roll The roll of the pose
     * \param[in] pitch The pitch of the pose
     * \param[in] yaw The yaw of the pose
     * \param[in] message A key word to help distinguish type of pose (e.g., approach, grasp, place)
     * \returns True if end effectors is successfully moved to goal pose
     */
    bool moveArmPTP ( const double &x, const double &y, const double &z,
                      const double &roll, const double &pitch, const double &yaw,
                      const std::string &message = "" );

    /**
     * \brief Moves the end effector to a pose in the map frame specified by a position and orientation (quaternion)
     * \param[in] x The x coordinate of pose
     * \param[in] y The y coordinate of pose
     * \param[in] z The z coordinate of pose
     * \param[in] xx The x component of the quaternion
     * \param[in] yy The y component of the quaternion
     * \param[in] zz The z component of the quaternion
     * \param[in] ww The w component of the quaternion
     * \param[in] message A key word to help distinguish type of pose (e.g., approach, grasp, place)
     * \returns True if end effectors is successfully moved to goal pose
     */
    bool moveArmPTP ( const double &x, const double &y, const double &z,
                      const double &xx, const double &yy, const double &zz, const double ww,
                      const std::string &message = "" );

    /**
     * \brief Moves the end effector to a pose in the map frame
     * \param[in] goal The goal pose
     * \param[in] message A key word to help distinguish type of pose (e.g., approach, grasp, place)
     * \returns True if end effectors is successfully moved to goal pose
     */
    bool moveArmPTP ( const geometry_msgs::PoseStamped &goal, const std::string &message = "" );

    /**
     * \brief Moves the base and arm to an 8DOF joint configuration
     *        Order is [base_jointx base_jointy base_jointz arm_joint1 arm_joint2 arm_joint3 arm_joint4 arm_joint5]
     * \param[in] joint_values The goal joint values for the base and arm
     * \param[in] message A key word to help distinguish type of pose (e.g., approach, grasp, place)
     * \returns True if joints are moved to their goal configuration
     */
    bool moveArmJoints ( const std::vector<double> &joint_values, const std::string &message = "" );

    /**
     * \brief Executes the planned joint trajectory by sending commands to the arm controller
     * \param[in] message A key word to help distinguish type of pose (e.g., approach, grasp, place)
     * \returns True if joints successfully follow the trajectory
     */
    bool sendCommandTrajectory ( const std::string &message = "" );

    /**
     * \brief Callback function to update the current joint state
     * \param[in] joints The message from joint state topic
     */
    void jointsStateCallBack ( const sensor_msgs::JointStateConstPtr &joints );

    /**
     * \brief Callback function to update the current command to the joints
     * \param[in] cmd The message from command topic
     */
    void jointsCommandCallBack ( const trajectory_msgs::JointTrajectoryConstPtr &cmd );

    /**
     * \brief Transforms a pose from one frame to another using the TF tree
     * \param[in] origin_frame The frame to transform from
     * \param[in] target_frame The frame to transform to
     * \param[in] in The pose to transform
     * \param[out] out The transformed pose
     * \returns True if the pose is successfully transformed
     */
    bool transformPose ( const std::string &origin_frame, const std::string &target_frame,
                         geometry_msgs::PoseStamped &in, geometry_msgs::PoseStamped &out ) const;

    /**
     * \brief Computes the difference between two poses
     * \param[in] pose1 The first vector of values
     * \param[in] pose2 The second vector of values
     * \returns A vector of values representing the element wise difference of the input vectors (as absolute values)
     */
    std::vector<double> poseDiff ( const std::vector<double> &pose1, const std::vector<double> &pose2 ) const;

    /**
     * \brief Checks if the arm is in the folded position
     * \returns True if all joint values match the folded joint values
     */
    bool armIsFolded () const;

    /**
     * \brief Checks if the arm is in the unfolded position
     * \returns True if all joint values match the unfolded joint values
     */
    bool armIsUnfolded () const;

    /**
     * \brief Waits until the current commanded trajectory is completed by the robot
     * \param[in] timeout The maximum amount of time to wait for trajectory completion before returning failure
     * \returns True if the robot completes the trajectory within the maximum allowed time
     */
    bool waitForTrajectoryCompletion ( const double &timeout = MAX_WAIT_TRAJECTORY_COMPLETION_);

    /**
     * \brief Publishes a visualization marker (as an arrow) of the goal end effector pose
     * \param[in] pose The end effector goal pose [x y z roll pitch yaw]
     */
    void publishGoalMarker ( const std::vector<double> &pose );

};

/**
 * \brief Appends the node name to the front of a string to help searching for parameters within the namespace
 * \param[in] str The string that the node name will be added to the front of
 * \returns A string with the node name added to the front of the input
 */
std::string addNodeName ( const std::string &str );

#endif

