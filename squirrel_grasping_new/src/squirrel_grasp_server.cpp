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


  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelGraspServer::graspCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
  ROS_INFO ( "[SquirrelGraspServer::graspCallBack] Called with parameters" );

  bool success = false;
  if ( hand_type_ == SquirrelGraspServer::METAHAND )
    success = metahandCallBack ( goal );
  else if ( hand_type_ == SquirrelGraspServer::SOFTHAND )
    success = softhandCallBack ( goal );

  if ( success )
  {
    ROS_INFO ( "[SquirrelGraspServer::graspCallBack] Succeeded" );
    // set the action state to succeeded
    as_.setSucceeded ( result_ );
  }
//    // helper variables
//    ros::Rate r(1);
//    bool success = true;

//    // push_back the seeds for the fibonacci sequence
//    feedback_.sequence.clear();
//    feedback_.sequence.push_back(0);
//    feedback_.sequence.push_back(1);

//    // publish info to the console for the user
//    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

//    // start executing the action
//    for(int i=1; i<=goal->order; i++)
//    {
//      // check that preempt has not been requested by the client
//      if (as_.isPreemptRequested() || !ros::ok())
//      {
//        ROS_INFO("%s: Preempted", action_name_.c_str());
//        // set the action state to preempted
//        as_.setPreempted();
//        success = false;
//        break;
//      }
//      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
//      // publish the feedback
//      as_.publishFeedback(feedback_);
//      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
//      r.sleep();
//    }
}

bool SquirrelGraspServer::metahandCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
  // Open hand
  feedback_.current_phase = "opening hand";
  feedback_.current_status = "success";

  // Plan arm to unfolded position
  // If in folded position, then use unfold service
  // Otherwise plan to unfolded position

  // Close hand

  // Plan arm to safe position
  return true;
}

bool SquirrelGraspServer::softhandCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
  feedback_.current_phase = "opening hand";
  feedback_.current_status = "success";
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
