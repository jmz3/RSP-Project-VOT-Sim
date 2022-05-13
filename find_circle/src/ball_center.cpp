/*
 * Name: Brandon Marlowe
 * ---------------------
 * TurtleBot detects and follows orange ball, always staying at least ~ 0.7
 * meters away
 */

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

#define RGB_FOCAL_LEN_MM 138.90625  // camera focal length in mm ... 525 pixels
#define BALL_DIAM_MM 200.0          // 9" diameter ball in mm
#define CAMERA_HEIGHT_MM 300.0      // height of camera off ground in mm
#define IMG_HEIGHT_PX 480.0         // in pixels
#define IMG_WIDTH_PX 640.0          // in pixels
#define MAX_BOT_VEL 0.65            // max speed TurtleBot is capable of
#define MIN_BOT_VEL 0.2             // the min speed I want the TurtleBot to go

static const std::string OPENCV_WINDOW = "Image window";
static double currObjDist = 0.0, alignmentError = 0.0, prevVelX = 0.0,
              botVelX = 0.0;
static int x, y, radius;

class DetectorTracker {
  ros::NodeHandle nodeHandle_;
  image_transport::ImageTransport imageTransport_;
  image_transport::Subscriber imageSub_;
  image_transport::Publisher imagePub_;
  ros::Subscriber odomSub_ = nodeHandle_.subscribe("/odom", 1000, odomCallback);

  ros::Publisher velPub = nodeHandle_.advertise<geometry_msgs::Twist>(
      "/cmd_vel_mux/input/teleop", 1);
  geometry_msgs::Twist twistMsg;

 public:
  DetectorTracker() : imageTransport_(nodeHandle_) {
    // Subscribe to input video feed and publish output video feed
    imageSub_ = imageTransport_.subscribe(
        "/usb_cam/image_raw", 10, &DetectorTracker::imageCallback, this);
    imagePub_ = imageTransport_.advertise("/image_converter/output_video", 10);

    cv::namedWindow(OPENCV_WINDOW, 0);
  }

  ~DetectorTracker() { cv::destroyWindow(OPENCV_WINDOW); }

  // calculates current currObjDist from object based on known height of object,
  // height of camera off ground and current perceived object size rounded to
  // nearest 10th
  double distFromObj(int objSize) {
    double distMeters = (((RGB_FOCAL_LEN_MM * BALL_DIAM_MM * IMG_HEIGHT_PX) /
                          (objSize * CAMERA_HEIGHT_MM)) /
                         1000);
    return floor((distMeters * 10 + 0.5)) / 10;
  }

  // publishes movement to robot based on currObjDist from ball and
  // approximately centers the robot's camera
  void moveTurtleBot() {
    // 225 worked well as a denominator to smooth the alignment
    twistMsg.angular.z = -alignmentError / 225.0;

    if (currObjDist < 0.7)
      twistMsg.linear.x = -0.5;
    else if (currObjDist > 1.0)
      twistMsg.linear.x =
          (0.3 *
           currObjDist);  // extremely simple proportional velocity controller
    else
      twistMsg.linear.x = 0.0;

    if (twistMsg.linear.x > MAX_BOT_VEL) twistMsg.linear.x = MAX_BOT_VEL;

    prevVelX = twistMsg.linear.x;

    velPub.publish(twistMsg);
  }

  static void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // robot linear and angular velocity rounded to the nearest 100th
    botVelX = floor((msg->twist.twist.linear.x * 100 + 0.5)) / 100;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvPtr;
    cv_bridge::CvImagePtr cvGrayPtr;
    std::vector<cv::Vec3f> circleIMG;
    cv::Mat hsvIMG, redIMG_lower, redIMG_upper, redIMG, srcIMG;

    cv::Scalar black =
        (0, 255, 5);  // RGB color for circle to be drawn on image
    cv::Scalar blue = (200, 200, 250);  // RGB color for text displayed on image

    try {
      cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      srcIMG = cvPtr->image;

      // converting color to HSV
      cv::cvtColor(srcIMG, hsvIMG, CV_BGR2HSV);

      // defining upper and lower red color range
      cv::inRange(hsvIMG, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255),
                  redIMG_lower);
      cv::inRange(hsvIMG, cv::Scalar(160, 100, 100), cv::Scalar(170, 255, 255),
                  redIMG_upper);

      // weighting image and performing blur to reduce noise in image
      cv::addWeighted(redIMG_lower, 1.0, redIMG_upper, 1.0, 0.0, redIMG);
      cv::GaussianBlur(redIMG, redIMG, cv::Size(9, 9), 2, 2);

      // Hough gradient transform to find circles
      cv::HoughCircles(redIMG, circleIMG, CV_HOUGH_GRADIENT, 1, hsvIMG.rows / 8,
                       100, 20, 0, 0);

    }

    catch (cv_bridge::Exception &exception) {
      ROS_ERROR("cv_bridge exception: %s", exception.what());
      return;
    }

    for (size_t i = 0; i < circleIMG.size(); i++) {
      // center coordinates of circle, and the radius
      x = static_cast<int>(round(circleIMG[i][0]));
      y = static_cast<int>(round(circleIMG[i][1]));
      radius = static_cast<int>(round(circleIMG[i][2]));
      cv::Point center(x, y);

      // draws circle around ball and cross-hair at center
      cv::circle(srcIMG, center, radius, black, 2);
      cv::line(srcIMG, center, center, black, 2);
      


      alignmentError = (x - (IMG_WIDTH_PX / 2.0));

      // prevents extreme distance estimation fluctuation
      if (i % 13 == 0) currObjDist = distFromObj(radius);

      moveTurtleBot();  // uses global variables to publish movement

      // text overlay of current currObjDist from ball and TurtleBot velocity
      std::stringstream ssDist, ssBotVelX, ssAlignError;
      ssAlignError << alignmentError;
      ssDist << currObjDist;
      ssBotVelX << botVelX;

      std::string alignErrStr = "    ALN_ERR: " + ssAlignError.str() + " px";
      std::string objDistStr = "    OBJ_DST: " + ssDist.str() + " m";
      std::string botVelStr = "    BOT_VEL: " + ssBotVelX.str() + " m/s";

      cv::putText(srcIMG, alignErrStr, cv::Point(350, 375),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
      cv::putText(srcIMG, objDistStr, cv::Point(350, 400),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
      cv::putText(srcIMG, botVelStr, cv::Point(350, 425),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
    }

    // image is published to screen with circle drawn around red ball
    cv::imshow(OPENCV_WINDOW, srcIMG);
    cv::waitKey(3);

    // Output modified video stream
    imagePub_.publish(cvPtr->toImageMsg());
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "ball_tracker_node");
  DetectorTracker detectorTracker;
  ros::spin();

  return 0;
}