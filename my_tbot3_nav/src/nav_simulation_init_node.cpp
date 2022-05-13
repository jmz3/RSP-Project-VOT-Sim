#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>


int sequence = 0;
ros::Publisher goal_pub;
double curr_ori_z;
double curr_ori_w;

void waypointCB(const geometry_msgs::Point msg){
    double x = msg.x;
    double y = msg.y;
    double z = msg.z;
    double ori_z[10] = {-0.207,-0.150,0.005,0.4043,0.659,0.753,0.949,1,1,-0.19};
    double ori_w[10] = {0.978,0.989,1,0.9146,0.752,0.658,0.316,0,0,0.9807};
    
    geometry_msgs::PoseStamped pose_stamped_msg;

    pose_stamped_msg.header.seq = sequence;
    pose_stamped_msg.header.stamp =  ros::Time::now();
    pose_stamped_msg.header.frame_id = "map";
    pose_stamped_msg.pose.position.x = x; 
    pose_stamped_msg.pose.position.y = y;
    pose_stamped_msg.pose.orientation.x = 0;
    pose_stamped_msg.pose.orientation.y = 0;

    if (z < 0.1) {
        pose_stamped_msg.pose.position.z = z;

        pose_stamped_msg.pose.orientation.z = ori_z[sequence];
        pose_stamped_msg.pose.orientation.w = ori_w[sequence];

        goal_pub.publish(pose_stamped_msg);

        sequence++;
        if (sequence == 10){
            sequence = 0;
        }
    } else {
        pose_stamped_msg.pose.position.z = 0;

        pose_stamped_msg.pose.orientation.z = curr_ori_z;
        pose_stamped_msg.pose.orientation.w = curr_ori_w;

        goal_pub.publish(pose_stamped_msg);
    }

}

void odomCB(const nav_msgs::Odometry msg){
    curr_ori_z = msg.pose.pose.orientation.z;    
    curr_ori_w = msg.pose.pose.orientation.w;    
}

int main(int argc, char** argv){

    ros::init(argc, argv, "robot_nav");
    ros::NodeHandle nh;

    ros::Publisher init_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",5);
    ros::Subscriber goal_sub = nh.subscribe("/fake_goal",1000,waypointCB);
    ros::Subscriber odom_sub = nh.subscribe("/odom",1000,odomCB);


    ros::service::waitForService("amcl/set_parameters");
    geometry_msgs::PoseWithCovarianceStamped init_pose_cov_stamp;
    init_pose_cov_stamp.header.seq = 1;
    init_pose_cov_stamp.header.stamp = ros::Time::now();
    init_pose_cov_stamp.header.frame_id = "map";


    init_pose_cov_stamp.pose.pose.position.x = -2;
    init_pose_cov_stamp.pose.pose.position.y = -0.6;
    init_pose_cov_stamp.pose.pose.position.z = 0;

    init_pose_cov_stamp.pose.pose.orientation.x = 0;
    init_pose_cov_stamp.pose.pose.orientation.y = 0;
    init_pose_cov_stamp.pose.pose.orientation.z = 0;
    init_pose_cov_stamp.pose.pose.orientation.w = 1;

    for (int i = 0; i < 36; i++){
        init_pose_cov_stamp.pose.covariance[i] = 0.0;
    }

    //init_covposepose.covariance 

    ros::Duration(1).sleep();
    init_pub.publish(init_pose_cov_stamp);

    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",5);

    ros::spin();
    
    return 0;
}

