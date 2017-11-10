#ifndef SQUIRREL_GRASP_SERVER_H_
#define SQUIRREL_GRASP_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_manipulation_msgs/BlindGraspAction.h>

#define METAHAND_STRING_ "metahand"
#define SOFTHAND_STRING_ "softhand"

/**
 * \brief Grasp server
 * \author Tim Patten (patten@acin.tuwien.ac.at)
 */
class SquirrelGraspServer
{
public:

  enum HandType
  {
    UNKNOWN = 0,
    METAHAND = 1,
    SOFTHAND = 2
  };

  /**
   * \brief Constructor with ros node handle
   */
  SquirrelGraspServer ( ros::NodeHandle &n, const std::string &action_name );

  /**
   * \brief Destructor
   */
  virtual
  ~SquirrelGraspServer ();

  /**
   * \brief Initialize the server with hand type and find servers and topics
   */
  bool initialize ( const std::string &action_name );

private:

  // Node handle
  ros::NodeHandle *n_;
  // Action client
  actionlib::SimpleActionServer<squirrel_manipulation_msgs::BlindGraspAction> as_;
  // Action name
  std::string action_name_;
  // Hand type
  HandType hand_type_;
  // Messages to publish feedback and result
  squirrel_manipulation_msgs::BlindGraspFeedback feedback_;
  squirrel_manipulation_msgs::BlindGraspResult result_;

  void graspCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

  bool metahandCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

  bool softhandCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal );

};

#endif

