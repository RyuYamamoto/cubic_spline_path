#include <ros/ros.h>
#include <cubic_spline_path/cubic_spline_path.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "cubic_spline_path_node");
  ros::NodeHandle nh;
  CubicSplinePath instance(nh);
  ros::spin();
  return 0;
}
