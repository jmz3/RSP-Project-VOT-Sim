#include <auto_move/auto_move.hpp>


AutoMove::AutoMove(ros::NodeHandle nodeHandle_){
//     ros::Subscriber odomSub_ = nodeHandle_.subscribe("/odom", 1000, odomCallback);
    lidarSub_ = nodeHandle_.subscribe("/scan", 1000, &AutoMove::moveTurtleBot, this);

    velPub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

}

AutoMove::~AutoMove(){}

// void AutoMove::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // robot linear and angular velocity rounded to the nearest 100th
    // botVelX = floor((msg->twist.twist.linear.x * 100 + 0.5)) / 100;
// }

void AutoMove::moveTurtleBot(const sensor_msgs::LaserScan::ConstPtr &lidar_msg) {
    
    geometry_msgs::Twist twistMsg;
    float thresh1 = 0.8;
    float thresh2 = 0.8;

    // std::cout << lidar_msg->ranges[0] <<std::endl;

    // check if obstacles in the frony and 15 degree left and right

    if (lidar_msg->ranges[0] > thresh1 && lidar_msg->ranges[15] > thresh2 &&  lidar_msg->ranges[345] > thresh2){
        
        // std::cout << "move turtle" <<std::endl;
        // go forward 
        twistMsg.linear.x = 0.1; 
        // do not rotate
        twistMsg.angular.z = 0.0;
    } else{

        // std::cout << "stop turtle" <<std::endl;
        // stop
        twistMsg.linear.x = 0.0;
        // rotate c-w
        twistMsg.angular.z = 0.1;

        if (lidar_msg->ranges[0] > thresh1 && lidar_msg->ranges[15] > thresh2 &&  lidar_msg->ranges[345] > thresh2){
            twistMsg.linear.x = 0.1;
            twistMsg.angular.z = 0.0;
        }
    }

    velPub.publish(twistMsg);
}

void AutoMove::stopTurtleBot(){
    geometry_msgs::Twist stopMSG;

    stopMSG.linear.x = 0.0;
    stopMSG.angular.z = 0.0;

    velPub.publish(stopMSG);

}
