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
#define MAX_WAIT_TRAJECTORY_COMPLETION_ 60.0
#define JOINT_IN_POSITION_THRESHOLD_ 0.139626 // 8 degrees
#define APPROACH_HEIGHT_ 0.15  // 15 cm

/**
 * \brief Object Manipulation server to perform grasping, placing, hand opening and closing
 * \author Tim Patten (patten@acin.tuwien.ac.at)
 */
class SquirrelObjectManipulationServer
{
	public:

		enum HandType
		{
			UNKNOWN_HAND = 0,
			METAHAND = 1,
			SOFTHAND = 2
		};

		enum ActionType
		{
			UNKNOWN_ACTION = 0,
			OPEN_HAND_ACTION = 1,
			CLOSE_HAND_ACTION = 2,
			GRASP = 3,
			PLACE = 4
		};

		enum MetahandActuations
		{
			DISABLE_MOTORS = 0,
			ENABLE_MOTORS = 1,
			CLOSE_METAHAND = 2,
			OPEN_METAHAND = 3,
			TO_LOWER = 4,
			TO_UPPER = 5,
			PRINT_STATE = 6,
			FOLD_METAHAND = 9
		};

		enum SofthandActuations
		{
			OPEN_SOFTHAND = 0,
			CLOSE_SOFTHAND = 1
		};

		/**
		 * \brief Constructor with ros node handle
		 */
		SquirrelObjectManipulationServer ( ros::NodeHandle &n, const std::string &action_name );

		/**
		 * \brief Destructor
		 */
		virtual	~SquirrelObjectManipulationServer ();

		/**
		 * \brief Initialize the server with hand type and find servers and topics
		 */
		bool initialize ( const std::string &action_name );

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
		kclhand_control::HandOperationMode hand_goal_kcl_;
		squirrel_manipulation_msgs::SoftHandGrasp hand_goal_softhand_;

		// Marker publisher
		ros::Publisher goal_pose_pub_;
		visualization_msgs::Marker goal_marker_;
		// Joint callback
		ros::Subscriber joints_state_sub_;
		std::vector<double> current_joints_;
		// Joints command callback
		ros::Subscriber joints_command_sub_;
		std::vector<double> current_cmd_;

		// Transform
		tf::TransformListener tf_listener_;

		// Important poses
		std::vector<double> folded_pose;
		std::vector<double> unfolded_pose;

		void actionServerCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

		bool handActuate ();

		bool metahandActuate ( const MetahandActuations &methand_action );

		bool softhandActuate ( const SofthandActuations &softhand_action );

		bool grasp ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

		bool metahandGrasp ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

		bool softhandGrasp ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

		bool place ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal);

		bool metahandPlace ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

		bool softhandPlace ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

		bool moveArmPTP ( const double &x, const double &y, const double &z,
				const double &roll, const double &pitch, const double &yaw,
				const std::string &message = "" );

		bool moveArmPTP ( const double &x, const double &y, const double &z,
				const double &xx, const double &yy, const double &zz, const double ww,
				const std::string &message = "" );

		bool moveArmPTP ( const geometry_msgs::PoseStamped &goal, const std::string &message = "" );

		bool moveArmJoints ( const std::vector<double> &joint_values, const std::string &message = "" );

		void jointsStateCallBack ( const sensor_msgs::JointStateConstPtr &joints );

		void jointsCommandCallBack ( const trajectory_msgs::JointTrajectoryConstPtr &cmd );

		bool transformPose ( const std::string &origin_frame, const std::string &target_frame,
				geometry_msgs::PoseStamped &in, geometry_msgs::PoseStamped &out ) const;

		std::vector<double> poseDiff ( const std::vector<double> &pose1, const std::vector<double> &pose2 ) const;

		bool armIsFolded () const;

		bool armIsUnfolded () const;

		bool waitForTrajectoryCompletion ( const double &timeout = MAX_WAIT_TRAJECTORY_COMPLETION_);

		void publishGoalMarker ( const std::vector<double> &pose );

};

std::string addNodeName ( const std::string &str );

#endif

