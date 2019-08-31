#include <ros/ros.h>

#include "aero_ctrl/beb_pos_pid.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "beb_pos_pid_node");

  ros::NodeHandle nh;

  ROS_INFO("Starting beb_pos_pid_node ...");
  beb_pos_pid::BebPosPid pos_ctrl(nh);

  pos_ctrl.Spin();
  return 0;
}
