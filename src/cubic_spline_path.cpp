#include <cubic_spline_path/cubic_spline_path.hpp>

CubicSplinePath::CubicSplinePath(ros::NodeHandle nh)
    : _nh(nh)
{
    ros::NodeHandle _pnh("~");
    _pnh.param<std::string>("waypoint_filename", _waypoint_filename, "waypoint.yaml");
	_pnh.param<double>("sampling_rate", _sampling_rate, 0.01);

    _waypoint_list = _load_waypoint();

    cubic_spline = new CubicSpline(0.01);
}

// waypointを読み込む
std::vector<geometry_msgs::Pose> CubicSplinePath::_load_waypoint()
{
	std::vector<geometry_msgs::Pose> waypoint_list;
	waypoint_list.clear();
	std::ifstream ifs(_waypoint_filename.c_str(), std::ifstream::in);
	if(!ifs.good()) {
		ROS_ERROR_STREAM("File not exist.");
		return waypoint_list;	
	}
	YAML::Node node;
	node = YAML::Load(ifs);
	const YAML::Node &wp_node_temp = node["waypoints"];
	const YAML::Node &wp_node = wp_node_temp ? &wp_node_temp : NULL;
	if(wp_node != NULL) {
		for(int index=0;index<wp_node->size();index++) {
			geometry_msgs::Pose waypoint;
			waypoint.position.x = (*wp_node)[index]["position"]["x"].as<double>();
			waypoint.position.y = (*wp_node)[index]["position"]["y"].as<double>();
			tf2::Quaternion quat;
			quat::setRPY(0.0, 0.0, (*wp_node)[index]["position"]["yaw"].as<double>());
			waypoint.orientation.w = quat.w();
			waypoint.orientation.x = quat.x();
			waypoint.orientation.y = quat.y();
			waypoint.orientation.z = quat.z();
			waypoint_list.push_back(waypoint);
		}
	}
	return waypoint_list;
}

// waypointを描画
void CubicSplinePath::_draw_marker()
{

}

// スプライン補間された経路を描画
void CubicSplinePath::_draw_line()
{

}