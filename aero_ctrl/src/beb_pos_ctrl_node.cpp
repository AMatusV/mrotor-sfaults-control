#include <ros/ros.h>

#include "aero_ctrl/beb_pos_ctrl.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "beb_pos_ctrl_node");

  ros::NodeHandle nh;

  ROS_INFO("Starting beb_pos_ctrl_node ...");
  beb_pos_ctrl::BebPosCtrl pos_ctrl(nh);

  pos_ctrl.Spin();
  return 0;
}
