#include "odom_predictor/odom_predictor.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_predictor_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  OdomPredictor odom_predictor(nh, nh_private);

  ros::spin();

  return 0;
}
