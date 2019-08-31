#include "aero_ctrl/filter_ext.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "filter_node");

  ros::NodeHandle nh;

  ROS_INFO("Starting filter_node ...");
  filter_ext::FilterExt filter(nh);

  filter.Spin();
  return 0;
}
