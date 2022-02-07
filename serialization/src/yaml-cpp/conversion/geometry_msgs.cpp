#include <moveit_serialization/yaml-cpp/conversion/geometry_msgs.h>

#include <moveit_serialization/yaml-cpp/conversion/std_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/utilities.h>

namespace YAML {

Node convert<geometry_msgs::Pose>::encode(const geometry_msgs::Pose& rhs) {
	Node node;
	node["position"] = rhs.position;
	node["orientation"] = rhs.orientation;
	return node;
}

bool convert<geometry_msgs::Pose>::decode(const Node& node, geometry_msgs::Pose& rhs) {
	rhs = geometry_msgs::Pose();

	if (node["position"])
		rhs.position = node["position"].as<geometry_msgs::Point>();

	if (node["orientation"])
		rhs.orientation = node["orientation"].as<geometry_msgs::Quaternion>();

	return true;
}

Node convert<geometry_msgs::PoseStamped>::encode(const geometry_msgs::PoseStamped& rhs) {
	Node node;
	if (!isHeaderEmpty(rhs.header))
		node["header"] = rhs.header;

	node["pose"] = rhs.pose;

	return node;
}

bool convert<geometry_msgs::PoseStamped>::decode(const Node& node, geometry_msgs::PoseStamped& rhs) {
	rhs = geometry_msgs::PoseStamped();

	if (node["header"])
		rhs.header = node["header"].as<std_msgs::Header>();
	else
		rhs.header = getDefaultHeader();

	rhs.pose = node["pose"].as<geometry_msgs::Pose>();

	return true;
}

Node convert<geometry_msgs::Transform>::encode(const geometry_msgs::Transform& rhs) {
	Node node;
	node["translation"] = rhs.translation;
	node["rotation"] = rhs.rotation;
	return node;
}

bool convert<geometry_msgs::Transform>::decode(const Node& node, geometry_msgs::Transform& rhs) {
	rhs = geometry_msgs::Transform();

	if (node["translation"])
		rhs.translation = node["translation"].as<geometry_msgs::Vector3>();

	if (node["rotation"])
		rhs.rotation = node["rotation"].as<geometry_msgs::Quaternion>();

	return true;
}
Node convert<geometry_msgs::TransformStamped>::encode(const geometry_msgs::TransformStamped& rhs) {
	Node node;

	if (!isHeaderEmpty(rhs.header))
		node["header"] = rhs.header;

	node["child_frame_id"] = rhs.child_frame_id;
	node["transform"] = rhs.transform;
	return node;
}

bool convert<geometry_msgs::TransformStamped>::decode(const Node& node, geometry_msgs::TransformStamped& rhs) {
	rhs = geometry_msgs::TransformStamped();

	if (node["header"])
		rhs.header = node["header"].as<std_msgs::Header>();
	else
		rhs.header = getDefaultHeader();

	if (node["child_frame_id"])
		rhs.child_frame_id = node["child_frame_id"].as<std::string>();

	if (node["transform"])
		rhs.transform = node["transform"].as<geometry_msgs::Transform>();

	return true;
}

Node convert<geometry_msgs::Point>::encode(const geometry_msgs::Point& rhs) {
	Node node;

	node["x"] = rhs.x;
	node["y"] = rhs.y;
	node["z"] = rhs.z;

	return node;
}

bool convert<geometry_msgs::Point>::decode(const Node& node, geometry_msgs::Point& rhs) {
	rhs = geometry_msgs::Point();

	if (node.IsSequence()) {
		rhs.x = node[0].as<double>();
		rhs.y = node[1].as<double>();
		rhs.z = node[2].as<double>();
	} else {
		rhs.x = node["x"].as<double>();
		rhs.y = node["y"].as<double>();
		rhs.z = node["z"].as<double>();
	}
	return true;
}

Node convert<geometry_msgs::Vector3>::encode(const geometry_msgs::Vector3& rhs) {
	Node node;

	node["x"] = rhs.x;
	node["y"] = rhs.y;
	node["z"] = rhs.z;

	return node;
}

bool convert<geometry_msgs::Vector3>::decode(const Node& node, geometry_msgs::Vector3& rhs) {
	rhs = geometry_msgs::Vector3();

	if (node.IsSequence()) {
		rhs.x = node[0].as<double>();
		rhs.y = node[1].as<double>();
		rhs.z = node[2].as<double>();
	} else {
		rhs.x = node["x"].as<double>();
		rhs.y = node["y"].as<double>();
		rhs.z = node["z"].as<double>();
	}

	return true;
}

Node convert<geometry_msgs::Quaternion>::encode(const geometry_msgs::Quaternion& rhs) {
	Node node;

	node["x"] = rhs.x;
	node["y"] = rhs.y;
	node["z"] = rhs.z;
	node["w"] = rhs.w;

	return node;
}

bool convert<geometry_msgs::Quaternion>::decode(const Node& node, geometry_msgs::Quaternion& rhs) {
	rhs = geometry_msgs::Quaternion();

	if (node.IsSequence()) {
		rhs.x = node[0].as<double>();
		rhs.y = node[1].as<double>();
		rhs.z = node[2].as<double>();
		rhs.w = node[3].as<double>();
	} else {
		rhs.x = node["x"].as<double>();
		rhs.y = node["y"].as<double>();
		rhs.z = node["z"].as<double>();
		rhs.w = node["w"].as<double>();
	}
	return true;
}

Node convert<geometry_msgs::Twist>::encode(const geometry_msgs::Twist& rhs) {
	Node node;
	node["linear"] = rhs.linear;
	node["angular"] = rhs.angular;
	return node;
}

bool convert<geometry_msgs::Twist>::decode(const Node& node, geometry_msgs::Twist& rhs) {
	rhs = geometry_msgs::Twist();

	rhs.linear = node["linear"].as<geometry_msgs::Vector3>();
	rhs.angular = node["angular"].as<geometry_msgs::Vector3>();

	return true;
}

Node convert<geometry_msgs::Wrench>::encode(const geometry_msgs::Wrench& rhs) {
	Node node;
	node["force"] = rhs.force;
	node["torque"] = rhs.torque;
	return node;
}

bool convert<geometry_msgs::Wrench>::decode(const Node& node, geometry_msgs::Wrench& rhs) {
	rhs = geometry_msgs::Wrench();

	rhs.force = node["force"].as<geometry_msgs::Vector3>();
	rhs.torque = node["torque"].as<geometry_msgs::Vector3>();

	return true;
}

}  // namespace YAML
