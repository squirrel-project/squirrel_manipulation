// ROS includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <squirrel_kclhand/kclhandConfig.h>

// ROS message includes
#include <squirrel_kclhand_msgs/Joints.h>
#include <squirrel_kclhand_msgs/SetJointTargetAction.h>
#include <squirrel_kclhand_msgs/TrackNewSetpointsAction.h>
#include <squirrel_kclhand_msgs/SetMaxJointVelocityAction.h>

// other includes
#include <kclhand_common.cpp>


class kclhand_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<squirrel_kclhand::kclhandConfig> server;
    dynamic_reconfigure::Server<squirrel_kclhand::kclhandConfig>::CallbackType f;

    ros::Publisher joint_states_out_;
    actionlib::SimpleActionServer<squirrel_kclhand_msgs::SetJointTargetAction> as_set_joint_controller_target_;
    actionlib::SimpleActionServer<squirrel_kclhand_msgs::TrackNewSetpointsAction> as_track_new_setpoints_;
    actionlib::SimpleActionServer<squirrel_kclhand_msgs::SetMaxJointVelocityAction> as_set_max_joint_velocity_;

    kclhand_data component_data_;
    kclhand_config component_config_;
    kclhand_impl component_implementation_;

    kclhand_ros() : np_("~")
    , as_set_joint_controller_target_(n_, "set_joint_controller_target", boost::bind(&kclhand_impl::callback_set_joint_controller_target_, &component_implementation_, _1, &as_set_joint_controller_target_), false)
    , as_track_new_setpoints_(n_, "track_new_setpoints", boost::bind(&kclhand_impl::callback_track_new_setpoints_, &component_implementation_, _1, &as_track_new_setpoints_), false)
    , as_set_max_joint_velocity_(n_, "set_max_joint_velocity", boost::bind(&kclhand_impl::callback_set_max_joint_velocity_, &component_implementation_, _1, &as_set_max_joint_velocity_), false)
    {
        f = boost::bind(&kclhand_ros::configure_callback, this, _1, _2);
        server.setCallback(f);
        as_set_joint_controller_target_.start();
        as_track_new_setpoints_.start();
        as_set_max_joint_velocity_.start();


        joint_states_out_ = n_.advertise<squirrel_kclhand_msgs::Joints>("joint_states_out", 1);

        np_.param("model", component_config_.model, (std::string)"SQUIRREL Three Fingered Hand");
        np_.param("Simulated_Joint_Pos_Gain", component_config_.Simulated_Joint_Pos_Gain, (double)0.1);
        np_.param("Simulated_Joint_Inertia", component_config_.Simulated_Joint_Inertia, (double)1.0);
        np_.param("Simulation_Sampling_Time", component_config_.Simulation_Sampling_Time, (double)0.02);
        np_.param("Simulated_Joint_Damping", component_config_.Simulated_Joint_Damping, (double)5.0);
        np_.param("Hand_IP_Address", component_config_.Hand_IP_Address, (std::string)"127.0.0.1");
        np_.param("Hand_Port", component_config_.Hand_Port, (int)55200);
    }

    void configure_callback(squirrel_kclhand::kclhandConfig &config, uint32_t level)
    {
        component_config_.model = config.model;
        component_config_.Simulated_Joint_Pos_Gain = config.Simulated_Joint_Pos_Gain;
        component_config_.Simulated_Joint_Inertia = config.Simulated_Joint_Inertia;
        component_config_.Simulation_Sampling_Time = config.Simulation_Sampling_Time;
        component_config_.Simulated_Joint_Damping = config.Simulated_Joint_Damping;
        component_config_.Hand_IP_Address = config.Hand_IP_Address;
        component_config_.Hand_Port = config.Hand_Port;
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_joint_states_out_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_joint_states_out_active)
            joint_states_out_.publish(component_data_.out_joint_states_out);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "kclhand");

    kclhand_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update()
    ros::spin();

    return 0;
}
