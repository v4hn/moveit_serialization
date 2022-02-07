#include <moveit_serialization/yaml-cpp/conversion/std_msgs.h>

namespace YAML {

Node convert<std_msgs::Header>::encode(const std_msgs::Header& rhs) {
	Node node;
	if (rhs.seq != 0)
		node["seq"] = rhs.seq;

	if (!rhs.stamp.isZero()) {
		node["stamp"]["secs"] = rhs.stamp.sec;
		node["stamp"]["nsecs"] = rhs.stamp.nsec;
	}

	if (rhs.frame_id != "world" && rhs.frame_id != "/world")
		node["frame_id"] = rhs.frame_id;

	return node;
}

bool convert<std_msgs::Header>::decode(const Node& node, std_msgs::Header& rhs) {
	rhs = std_msgs::Header();
	rhs.frame_id = "world";

	if (node["seq"])
		rhs.seq = node["seq"].as<int>();

	if (node["stamp"]) {
		try {
			rhs.stamp.sec = node["stamp"]["sec"].as<int>();
			rhs.stamp.nsec = node["stamp"]["nsec"].as<int>();
		} catch (YAML::InvalidNode& e) {
			rhs.stamp.sec = node["stamp"]["secs"].as<int>();
			rhs.stamp.nsec = node["stamp"]["nsecs"].as<int>();
		}
	}

	if (node["frame_id"])
		rhs.frame_id = node["frame_id"].as<std::string>();

	return true;
}

Node convert<std_msgs::ColorRGBA>::encode(const std_msgs::ColorRGBA& rhs) {
	Node node;
	node.SetStyle(EmitterStyle::Flow);

	node.push_back(rhs.r);
	node.push_back(rhs.g);
	node.push_back(rhs.b);
	node.push_back(rhs.a);
	return node;
}

bool convert<std_msgs::ColorRGBA>::decode(const Node& node, std_msgs::ColorRGBA& rhs) {
	rhs = std_msgs::ColorRGBA();

	rhs.r = node[0].as<double>();
	rhs.g = node[1].as<double>();
	rhs.b = node[2].as<double>();
	rhs.a = node[3].as<double>();
	return true;
}

}  // namespace YAML
