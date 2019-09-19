#ifndef _CUBIC_SPLINE_H_
#define _CUBIC_SPLINE_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2/tf2_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <yaml-cpp/yaml.h>
#include <cubic_spline_path/cubic_spline.hpp>

class CubicSplinePath {
public:
    CubicSplinePath(ros::NodeHandle nh);
    ~CubicSplinePath(){}
    void run();
private:
    std::vector<geometry_msgs::Pose> _load_waypoint();
    void _draw_marker();
    void _draw_line();

    ros::NodeHandle _nh;
    ros::Publisher _path_pub;
    geometry_msgs::Pose _waypoint_list;
    std::vector<double> _waypoint_x, _waypoint_y;

    double _sampling_rate;
    std::string _waypoint_filename;
    CubicSpline *cubic_spline;
};

#endif