#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class AutoMove
{

private:
    ros::NodeHandle nodeHandle_;

    // ros::Subscriber odomSub_;
    ros::Subscriber lidarSub_;
    ros::Publisher velPub;
    // geometry_msgs::Twist twistMsg;

public:
    void moveTurtleBot(const sensor_msgs::LaserScan::ConstPtr &lidar_msg);
    void stopTurtleBot();
    AutoMove(ros::NodeHandle nodeHandle_);
    ~AutoMove();
};