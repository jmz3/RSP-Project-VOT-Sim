#include <find_circle/find_center.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "find_center");

  ros::NodeHandle nodeHandle_;

  FindCenter getcenter(nodeHandle_);
  
  ros::spin();

  return 0;
}