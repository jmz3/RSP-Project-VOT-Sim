#include <ros/ros.h>
#include <ros/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <my_tbot3_workcontrol/ControllerConfig.h>
#include <geometry_msgs/Twist.h>


class controller{

private:

    ros::NodeHandle nh;
    //ros::Publisher publisher;
    
    dynamic_reconfigure::Server<my_tbot3_workcontrol::ControllerConfig> my_controller;
    dynamic_reconfigure::Server<my_tbot3_workcontrol::ControllerConfig>::CallbackType callback;

    geometry_msgs::Twist stopMSG;
    ros::Publisher cmd_vel_pub;

    ros::Rate rate;

public:

    controller(ros::NodeHandle& nh);
    //void publish();
    void controller_config(my_tbot3_workcontrol::ControllerConfig& config, uint32_t level);

};