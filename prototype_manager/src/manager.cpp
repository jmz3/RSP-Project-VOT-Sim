#include <prototype_manager/manager.hpp>

manager::manager(ros::NodeHandle &nh) : nh(nh),
                                              rate(0.1)
{
    callback = boost::bind(&manager::manager_config, this, _1, _2);
    manager_server.setCallback(callback);

    finalposeSub_ = nh.subscribe("/odom",
                                 10,
                                 &manager::finalposeCB,
                                 this);
}

void manager::finalposeCB(const nav_msgs::Odometry &odom)
{
    p_x = odom.pose.pose.position.x;
    p_y = odom.pose.pose.position.y;
}

void manager::manager_config(prototype_manager::ManagerConfig &config, uint32_t level)
{
    if ((config.Switch_to_Navigation == 1) && (level == 0))
    {

        nh.setParam("stop_auto_move", 2);
        nh.setParam("/final_position/x", p_x);
        nh.setParam("/final_position/y", p_y);
        system("rosparam dump /tmp/stop_position.yaml /final_position");

        std::cout << "\n###################################################################\n"
                  << "#############  Switch to Navigation Mode  #########################\n"
                  << "###################################################################\n";
        ROS_INFO("Saving map...");
        ROS_INFO("Please launch start_nav_with_gz.launch for navigation");
        ROS_INFO("SLAM nodes shutting down......");
    }
}