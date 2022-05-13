#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <prototype_manager/ManagerConfig.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class manager
{

private:

    ros::NodeHandle nh;
    ros::Subscriber finalposeSub_;
    ros::Rate rate;

    double p_x, p_y;

    dynamic_reconfigure::Server<prototype_manager::ManagerConfig> manager_server;
    dynamic_reconfigure::Server<prototype_manager::ManagerConfig>::CallbackType callback;

public:

    manager(ros::NodeHandle &nh);
    ~manager(){}
    void finalposeCB(const nav_msgs::Odometry &odom);
    void manager_config(prototype_manager::ManagerConfig& config, uint32_t level);
};