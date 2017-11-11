#include "squirrel_grasping_new/squirrel_grasp_server.h"

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SquirrelGraspServer::SquirrelGraspServer ( ros::NodeHandle &n, const std::string &action_name ) :
  n_ ( new ros::NodeHandle(n) ),
  action_name_ ( action_name ),
  as_ ( *n_, action_name, boost::bind(&SquirrelGraspServer::graspCallBack, this, _1), false)
{
  as_.start();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SquirrelGraspServer::~SquirrelGraspServer ()
{}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelGraspServer::initialize ( const std::string &hand_name )
{
  if ( hand_name.compare(METAHAND_STRING_) == 0 )
  {
    hand_type_ = SquirrelGraspServer::METAHAND;
    ROS_INFO ( "[SquirrelGraspServer::initialize] Metahand selected" );
  }
  else if ( hand_name.compare(SOFTHAND_STRING_) == 0 )
  {
    hand_type_ = SquirrelGraspServer::SOFTHAND;
    ROS_INFO ( "[SquirrelGraspServer::initialize] Softhand selected" );
  }
  else
  {
    hand_type_ = SquirrelGraspServer::UNKNOWN;
    ROS_ERROR ( "[SquirrelGraspServer::initialize] Unknown hand name %s ", hand_name.c_str() );
    return false;
  }

  arm_unfold_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::UnfoldArm>("/squirrel_8dof_planning/unfold_arm") );
  arm_end_eff_planner_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::PlanEndEffector>("/squirrel_8dof_planning/find_plan_end_effector") );
  arm_pose_planner_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::PlanPose>("/squirrel_8dof_planning/find_plan_pose") );
  arm_send_trajectory_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::SendControlCommand>("/squirrel_8dof_planning/send_trajectory_controller") );
  //hand_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::PlanEndEffector>("/squirrel_8dof_planning/find_plan_end_effector") );

  // Get the folded and unfolded poses from the parameter server
  vector<double> loaded_poses;
  if ( !n_->getParam("/squirrel_grasp_server/trajectory_folding_arm", loaded_poses) )
  {
    ROS_ERROR ( "[SquirrelGraspServer::initialize] Could not retrieve unfolded poses " );
    return false;
  }

  if ( loaded_poses.size() % 5 == 0 )
  {
    // Folded pose is the first 5 values
    folded_pose.resize ( 5 );
    folded_pose[0] = loaded_poses[0];
    folded_pose[1] = loaded_poses[1];
    folded_pose[2] = loaded_poses[2];
    folded_pose[3] = loaded_poses[3];
    folded_pose[4] = loaded_poses[4];
    // Unfolded pose is the last 5 values
    int num_vals = loaded_poses.size();
    unfolded_pose.resize ( 5 );
    unfolded_pose[0] = loaded_poses[num_vals-5];
    unfolded_pose[1] = loaded_poses[num_vals-4];
    unfolded_pose[2] = loaded_poses[num_vals-3];
    unfolded_pose[3] = loaded_poses[num_vals-2];
    unfolded_pose[4] = loaded_poses[num_vals-1];
  }
  else
  {
    ROS_ERROR ( "[SquirrelGraspServer::initialize] Parameter list 'trajectory_folding_arm' is not divisible by 5, folding arm trajectory has not been loaded" );
    return false;
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelGraspServer::graspCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
  ROS_INFO ( "[SquirrelGraspServer::graspCallBack] Called with parameters" );

  bool success = false;

  // Transform to hand_wrist_link frame
  geometry_msgs::PoseStamped transformed_goal;
  string goal_frame = goal->heap_center_pose.header.frame_id;
  geometry_msgs::PoseStamped goal_pose = goal->heap_center_pose;
  //transformPose ( goal_frame, PLANNING_FRAME_, goal_pose, transformed_goal );

  if ( hand_type_ == SquirrelGraspServer::METAHAND )
    success = metahandCallBack ( transformed_goal );
  else if ( hand_type_ == SquirrelGraspServer::SOFTHAND )
    success = softhandCallBack ( transformed_goal );

  if ( success )
  {
    ROS_INFO ( "[SquirrelGraspServer::graspCallBack] Succeeded" );
    // set the action state to succeeded
    as_.setSucceeded ( result_ );
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelGraspServer::metahandCallBack ( const geometry_msgs::PoseStamped &goal )
{
  // Open hand
  feedback_.current_phase = "opening hand";
  feedback_.current_status = "starting";
  as_.publishFeedback ( feedback_ );
  // Call service ...
  feedback_.current_status = "success";
  as_.publishFeedback ( feedback_ );

  // Plan arm to unfolded position
  feedback_.current_phase = "unfolding";
  feedback_.current_status = "starting";
  as_.publishFeedback ( feedback_ );
  unfold_goal_.request.check_octomap_collision = true;
  unfold_goal_.request.check_self_collision = true;
  if ( !arm_unfold_client_->call(unfold_goal_) )
  {
    ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Failed to unfold the arm using unfolding service" );
    // Set the result to something here
    feedback_.current_status = "failed";
    return false;
  }
  // If in folded position, then use unfold service
  // Otherwise plan to unfolded position
  feedback_.current_status = "success";
  as_.publishFeedback ( feedback_ );

  // Plan end effector to goal position
  //rosservice call /squirrel_8dof_planning/find_plan_end_effector "check_octomap_collision: true
  feedback_.current_phase = "grasping";
  feedback_.current_status = "starting";
  as_.publishFeedback ( feedback_ );
  vector<double> goal_pose ( 6 );  // [x y z roll pitch yaw]
  goal_pose[0] = goal.pose.position.x;
  goal_pose[1] = goal.pose.position.y;
  goal_pose[2] = goal.pose.position.z;
  goal_pose[3] = 0;
  goal_pose[4] = 0;
  goal_pose[5] = 0;
  end_eff_goal_.request.positions = goal_pose;
  end_eff_goal_.request.max_planning_time = 3.0;
  end_eff_goal_.request.check_octomap_collision = true;
  end_eff_goal_.request.check_self_collision = true;
  end_eff_goal_.request.fold_arm = false;
  end_eff_goal_.request.min_distance_before_folding = 0.0;
  if ( !arm_unfold_client_->call(unfold_goal_) )
  {
    ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Failed to find a plan to pose [%.2f %.2f %.2f %.2f %.2f %.2f]",
               goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, 0.0, 0.0, 0.0 );
    // Set the result to something here
    feedback_.current_status = "failed";
    return false;
  }
  feedback_.current_status = "success";
  as_.publishFeedback ( feedback_ );

  // Close hand
  feedback_.current_phase = "closing hand";
  feedback_.current_status = "starting";
  as_.publishFeedback ( feedback_ );
  // Call service ...
  feedback_.current_status = "success";
  as_.publishFeedback ( feedback_ );

  // Plan arm to safe position
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelGraspServer::softhandCallBack ( const geometry_msgs::PoseStamped &goal )
{
  feedback_.current_phase = "opening hand";
  feedback_.current_status = "success";
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelGraspServer::transformPose ( const string &origin_frame, const string &target_frame,
                                          geometry_msgs::PoseStamped &in, geometry_msgs::PoseStamped &out )
{
  // Set the frames
  in.header.frame_id = origin_frame;
  out.header.frame_id = target_frame;
  // Transform using the listerner
  try
  {
    tf_listener_.transformPose ( target_frame, in, out );
  }
  catch ( tf::TransformException ex )
  {
    // Error occured!
    ROS_ERROR ( "[SquirrelGraspServer::transformPose] Tf listener exception thrown with message '%s'",ex.what() );
    ros::Duration(1.0).sleep();
    return false;
  }
  // Print out successful transformation
  ROS_INFO ( "[SquirrelGraspServer::transformPose] Transformed from: \n[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
             in.pose.position.x, in.pose.position.y, in.pose.position.z, in.pose.orientation.x, in.pose.orientation.y, in.pose.orientation.z, in.pose.orientation.w,
             out.pose.position.x, out.pose.position.y, out.pose.position.z, out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w );
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "squirrel_grasp_server" );
  ros::NodeHandle n ( "" );

  string action_name = "squirrel_grasp_server";
  if ( !n.getParam("action_name", action_name) )
    ROS_WARN ( "[SquirrelGraspServer] No input action name, using default %s", action_name.c_str() );
  string hand_name = "metahand";
  if ( !n.getParam("hand", hand_name) )
    ROS_WARN ( "[SquirrelGraspServer] No input hand name, using default %s", hand_name.c_str() );

  ROS_INFO ( "Starting squirrel grasp server with parameters: action_name = %s, hand_name = %s", action_name.c_str(), hand_name.c_str() );

  // Create segmenter
  SquirrelGraspServer grasp_server ( n, action_name );
  if ( !grasp_server.initialize (hand_name) )
  {
    ROS_WARN ( "[SquirrelGraspServer] Could not initialize" );
    ros::shutdown ;
    return EXIT_FAILURE;
  }
  // Otherwise listen to action calls
  ros::spin();

  // Shutdown and exit
  ros::shutdown();
  return EXIT_SUCCESS;
}
