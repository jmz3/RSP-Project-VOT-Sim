
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <tf/transform_broadcaster.h>

class FindCenter
{

private:
    ros::NodeHandle nodeHandle_;

    ros::Subscriber poseSub_;
    ros::Publisher posePub_;
    image_transport::ImageTransport imageTransport_;
    image_transport::Subscriber imageSub_;
    image_transport::Publisher imagePub_;

    // static const std::string OPENCV_WINDOW = "Image window";
    // static int x, y, radius;

    double r_real;
    double fx, fy;
    double focal_distance;
    double depth;

    Eigen::Vector3d t_base2cam;
    Eigen::Matrix3d M_base2cam;

    Eigen::Vector3d t_odom2base;
    Eigen::Matrix3d M_odom2base;

    Eigen::Matrix3d M_intrinsic;
    Eigen::Vector3d Pixel_Coordinates;
    Eigen::Vector3d Image_Coordinates;
    Eigen::Vector3d World_Coordinates;

    bool TF_RECEIVED;
    tf::TransformBroadcaster broadcaster_;
    tf::Transform transform_;
    geometry_msgs::Point ball_position;

public:
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void poseCallback(const nav_msgs::Odometry &odom);

    FindCenter(ros::NodeHandle nodeHandle_);
    ~FindCenter();
};