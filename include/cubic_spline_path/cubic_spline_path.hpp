#ifndef _CUBIC_SPLINE_H_
#define _CUBIC_SPLINE_H_

#include <fstream>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>
#include <cubic_spline_path/cubic_spline.hpp>

class CubicSplinePath {
public:
    CubicSplinePath(ros::NodeHandle nh);
    ~CubicSplinePath(){}
    void run();
private:
    std::vector<geometry_msgs::Pose> _load_waypoint();
    void _draw_marker(std::vector<geometry_msgs::Pose> waypoint_list);
    void _draw_line();

    ros::NodeHandle _nh;
    ros::Publisher _waypoint_marker_pub;
    ros::Publisher _path_pub;
    std::vector<geometry_msgs::Pose> _waypoint_list;
    std::vector<double> _waypoint_x, _waypoint_y;
    std::vector<double> _spline_path_x, _spline_path_y;

    double _sampling_rate;
    std::string _waypoint_filename;
    CubicSpline *cubic_spline;
};

#endif