#include <auto_move/auto_move.hpp>
#include <std_msgs/Bool.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "auto_move");

    ros::NodeHandle nodeHandle_;

    AutoMove auto_move(nodeHandle_);

    int stop_auto_move = 0; // 0 for keep running
                            // 1 for stop and shutdown
                            // 2 for stop, save map and shutdown

    nodeHandle_.setParam("stop_auto_move", stop_auto_move);

    geometry_msgs::Twist stopMSG;
    stopMSG.linear.x = 0.0;
    stopMSG.angular.z = 0.0;

    ros::Publisher stopPub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    while (nodeHandle_.getParam("stop_auto_move", stop_auto_move))
    {
        if (stop_auto_move == 1)
        {
            auto_move.~AutoMove();

            break;
        }

        else if (stop_auto_move == 2)
        {
            system("rosrun map_server map_saver -f /tmp/map");

            auto_move.~AutoMove();
            // ROS_INFO("destruct auto_move class object");
            // ROS_INFO("create a publisher");
            // ROS_INFO("send stop message");
            break;

            // nodeHandle_.shutdown();
        }

        ros::spinOnce();
    }
    // send stop velocity to cmd_vel
    stopPub.publish(stopMSG);
    ros::Duration(2).sleep();
    nodeHandle_.deleteParam("stop_auto_move");


    ROS_INFO("shutdown this node");
    nodeHandle_.shutdown();
    ROS_INFO("shutdown complete");

    return 0;
}
