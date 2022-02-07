#include <moveit_serialization/yaml-cpp/conversion/ros.h>

namespace YAML {

Node convert<ros::Time>::encode(const ros::Time& rhs) {
	Node node;
	node = rhs.toSec();
	return node;
}

bool convert<ros::Time>::decode(const Node& node, ros::Time& rhs) {
	rhs.fromSec(node.as<double>());
	return true;
}

Node convert<ros::Duration>::encode(const ros::Duration& rhs) {
	Node node;
	node = rhs.toSec();
	return node;
}

bool convert<ros::Duration>::decode(const Node& node, ros::Duration& rhs) {
	rhs.fromSec(node.as<double>());
	return true;
}

}  // namespace YAML
