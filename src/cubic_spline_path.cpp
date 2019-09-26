#include <cubic_spline_path/cubic_spline_path.hpp>

CubicSplinePath::CubicSplinePath(ros::NodeHandle nh)
	: _nh(nh)
{
	ros::NodeHandle _pnh("~");
	_pnh.param<std::string>("waypoint_filename", _waypoint_filename, "waypoint.yaml");
	_pnh.param<double>("sampling_rate", _sampling_rate, 0.01);

	// yamlファイルを読み込みwaypoint情報を格納する
	_waypoint_list = _load_waypoint();
	// x, yそれぞれ分けて格納する
	for(auto i : _waypoint_list)
	{
		_waypoint_x.push_back(i.position.x);
		_waypoint_y.push_back(i.position.y);
	}
	
	_waypoint_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("/waypoint", 10, true);

	/*
	cubic_spline = new CubicSpline(0.01);
	cubic_spline->init(_waypoint_y);

	// cubic splineによる補間を実行
	for(double i=_waypoint_x.front(); i<=_waypoint_x.back();i+_sampling_rate)
	{
		_spline_path_x.push_back(i);
		_spline_path_y.push_back(cubic_spline->calc(i));
	}
	*/
	_draw_marker(_waypoint_list);
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
	const YAML::Node *wp_node = wp_node_temp ? &wp_node_temp : NULL;
	if(wp_node != NULL) {
		for(int index=0;index<wp_node->size();index++) {
			geometry_msgs::Pose waypoint;
			waypoint.position.x = (*wp_node)[index]["position"]["x"].as<double>();
			waypoint.position.y = (*wp_node)[index]["position"]["y"].as<double>();
			tf2::Quaternion quat;
			quat.setRPY(0.0, 0.0, (*wp_node)[index]["position"]["yaw"].as<double>());
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
void CubicSplinePath::_draw_marker(std::vector<geometry_msgs::Pose> waypoint_list)
{
	visualization_msgs::MarkerArray marker_list;
	visualization_msgs::Marker marker;

	int id=0;

	for(std::size_t index=0;index<waypoint_list.size();index++)
	{
		marker.id = id;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		
		marker.scale.x = 0.3;
		marker.scale.y = 0.3;
		marker.scale.z = 0.3;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.ns = "waypoint";
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose = waypoint_list[id];
		marker_list.markers.push_back(marker);
		id++;
	}
	_waypoint_marker_pub.publish(marker_list);
}

// スプライン補間された経路を描画
void CubicSplinePath::_draw_line()
{

}
