#include <find_circle/find_center.hpp>

static const std::string OPENCV_WINDOW = "Image window";
static int x, y, radius;

// camera matrix
//  961.720459 0.000000 634.032689
//  0.000000 960.267296 486.452286
//  0.000000 0.000000 1.000000
//  focal length 3.04mm

FindCenter::FindCenter(ros::NodeHandle nodeHandle_) : imageTransport_(nodeHandle_),
                                                      TF_RECEIVED(false),
                                                      r_real(3.4e-2),
                                                      fx(961.720459),
                                                      fy(960.267296),
                                                      focal_distance(3.04e-3)
{
  // r_real = 3.4e-2;
  // fx = 961.720459, fy = 960.267296;
  // focal_distance = 3.04e-3;
  ball_position.x = 0.0;
  ball_position.y = 0.0;
  ball_position.z = 0.0;
  M_intrinsic << 961.720459, 0.000000, 634.032689,
      0.000000, 960.267296, 486.452286,
      0.000000, 0.000000, 1.000000;

  Eigen::Quaterniond q_base2cam(0.5, -0.5, 0.5, -0.5);
  t_base2cam << 0.076, 0.000, 0.103;
  M_base2cam = q_base2cam.matrix();

  // subscribe the transformation from odom to base_footprint
  //( or base_link, they are the same)
  poseSub_ = nodeHandle_.subscribe("/odom",
                                   10,
                                   &FindCenter::poseCallback,
                                   this);

  // Subscribe to input video feed 
  imageSub_ = imageTransport_.subscribe("/raspicam_node/image",
                                        10,
                                        &FindCenter::imageCallback,
                                        this);

  // publish output video feed
  imagePub_ = imageTransport_.advertise("/image_converter/output_video", 10);

  // publish the ball position
  posePub_ = nodeHandle_.advertise<geometry_msgs::Point>("/ball_position", 10);

  // Open a visualization window
  cv::namedWindow(OPENCV_WINDOW, 0);
}

FindCenter::~FindCenter() { cv::destroyWindow(OPENCV_WINDOW); }

void FindCenter::poseCallback(const nav_msgs::Odometry &odom)
{

  // get the current translation and rotation (in quaternion)
  Eigen::Quaterniond q_odom2base(odom.pose.pose.orientation.w,
                                 odom.pose.pose.orientation.x,
                                 odom.pose.pose.orientation.y,
                                 odom.pose.pose.orientation.z);

  t_odom2base << odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z;

  // convert it into rotation matrix
  M_odom2base = q_odom2base.toRotationMatrix();

  TF_RECEIVED = true;
}

void FindCenter::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cvPtr;
  std::vector<cv::Vec3f> circleIMG;
  cv::Mat hsvIMG, redIMG_lower, redIMG_upper, redIMG, srcIMG;

  // RGB color for circle to be drawn on image
  cv::Scalar black = (0, 255, 5);
  // RGB color for text displayed on image
  cv::Scalar blue = (200, 200, 250);

  try
  {
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    srcIMG = cvPtr->image;

    cv::medianBlur(srcIMG, srcIMG, 3);
    // converting color to HSV
    cv::cvtColor(srcIMG, hsvIMG, CV_BGR2HSV);

    // defining upper and lower red color range
    cv::inRange(hsvIMG, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), redIMG_lower);
    cv::inRange(hsvIMG, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), redIMG_upper);

    // weighting image and performing blur to reduce noise in image
    cv::addWeighted(redIMG_lower, 1.0, redIMG_upper, 1.0, 0.0, redIMG);
    cv::GaussianBlur(redIMG, redIMG, cv::Size(9, 9), 2, 2);

    // Hough gradient transform to find circles
    cv::HoughCircles(redIMG, circleIMG, CV_HOUGH_GRADIENT, 1, hsvIMG.rows / 8, 100, 20, 0, 0);
  }

  catch (cv_bridge::Exception &exception)
  {
    ROS_ERROR("cv_bridge exception: %s", exception.what());
    return;
  }

  // store the center coordinates and the radius (wrt pixel coordinate frame)
  int tmp_r = 0;
  cv::Point tmp_center(0, 0);
  for (size_t i = 0; i < circleIMG.size(); i++)
  {
    // center coordinates of circle, and the radius
    x = static_cast<int>(round(circleIMG[i][0]));
    y = static_cast<int>(round(circleIMG[i][1]));
    radius = static_cast<int>(round(circleIMG[i][2]));
    cv::Point center(x, y);
    // std::cout << "x:" << x << std::endl;

    if (radius >= tmp_r)
    {
      tmp_r = radius;
      tmp_center.x = x;
      tmp_center.y = y;

      // Estimate depth
      depth = 0.5 * (fx + fy) * (r_real / tmp_r);
      std::cout << "Pixel Radius:" << radius << std::endl;
      std::cout << "Depth : " << depth << std::endl;
    }
  }

  cv::circle(srcIMG, tmp_center, tmp_r, black, 2);

  // Calculate the center coordinate wrt camera frame and world frame
  if (TF_RECEIVED == true)
  {
    Pixel_Coordinates << (double)tmp_center.x, (double)tmp_center.y, 1.0;

    // Cartesian coordinates in [x/z, y/z ,1] form
    Image_Coordinates = M_intrinsic.inverse() * Pixel_Coordinates; 
    // from [x/z, y/z ,1] to [x,y,z]
    Image_Coordinates = depth * Image_Coordinates;

    // the ball position                  
    World_Coordinates = M_odom2base * (M_base2cam * Image_Coordinates + t_base2cam) + t_odom2base;

    std::cout << "Image Coordinates: \n"
              << Image_Coordinates << std::endl;
    std::cout << "World Coordinates: \n"
              << World_Coordinates << std::endl;

    // send this frame to the tf tree
    // to visualize it in the rviz
    transform_.setOrigin(tf::Vector3(Image_Coordinates[0],
                                     Image_Coordinates[1],
                                     Image_Coordinates[2]));

    transform_.setRotation(tf::Quaternion(0, 0, 0, 1));

    broadcaster_.sendTransform(tf::StampedTransform(transform_,
                                                    ros::Time::now(),
                                                    "camera_rgb_optical_frame",
                                                    "ball_frame"));

    // publish the ball position to the /ball_position topic
    ball_position.x = World_Coordinates[0];
    ball_position.y = World_Coordinates[1];
    posePub_.publish(ball_position);
  }
  else
  {
    ROS_WARN("No tf received.");
  }

  // image is published to screen with circle drawn around red ball
  cv::imshow(OPENCV_WINDOW, srcIMG);
  cv::waitKey(3);

  // Output modified video stream
  imagePub_.publish(cvPtr->toImageMsg());
}