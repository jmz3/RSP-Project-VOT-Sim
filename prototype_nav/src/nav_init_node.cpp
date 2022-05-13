#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char **argv)
{

    // system("roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml");
    ros::init(argc, argv, "robot_srv");
    ros::NodeHandle nh;
    bool start_flag;
    if (nh.getParam("/Start_Nav", start_flag))
    {
        ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 5);

        ros::service::waitForService("amcl/set_parameters");
        geometry_msgs::PoseWithCovarianceStamped init_pose_cov_stamp;
        init_pose_cov_stamp.header.seq = 1;
        init_pose_cov_stamp.header.stamp = ros::Time::now();
        init_pose_cov_stamp.header.frame_id = "map";

        init_pose_cov_stamp.pose.pose.position.x = 0.3714;
        init_pose_cov_stamp.pose.pose.position.y = 1.207;
        init_pose_cov_stamp.pose.pose.position.z = 0;

        init_pose_cov_stamp.pose.pose.orientation.x = 0;
        init_pose_cov_stamp.pose.pose.orientation.y = 0;
        init_pose_cov_stamp.pose.pose.orientation.z = 0.9122;
        init_pose_cov_stamp.pose.pose.orientation.w = 0.4097;

        for (int i = 0; i < 36; i++)
        {
            init_pose_cov_stamp.pose.covariance[i] = 0.0;
        }

        // init_covposepose.covariance

        ros::Duration(1).sleep();
        pub.publish(init_pose_cov_stamp);

        std::cout << "good so far" << std::endl;
    }
        ros::spin();
    

    return 0;
}