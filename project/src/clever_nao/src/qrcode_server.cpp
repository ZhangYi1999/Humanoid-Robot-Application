#include "ros/ros.h"
#include "clever_nao/QRCode.h"

bool service(clever_nao::QRCode::Request  &req,clever_nao::QRCode::Response &res)
{

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qrcode_server");
  ros::NodeHandle n;

//   ros::ServiceServer service = n.advertiseService("add_two_ints", add);
//   ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}