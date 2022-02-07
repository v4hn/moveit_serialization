#include <moveit_serialization/yaml-cpp/conversion/octomap_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/std_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/geometry_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/utilities.h>

namespace YAML {

Node convert<octomap_msgs::Octomap>::encode(const octomap_msgs::Octomap& rhs) {
	Node node;

	if (!isHeaderEmpty(rhs.header))
		node["header"] = rhs.header;

	node["binary"] = boolToString(rhs.binary);
	node["id"] = rhs.id;
	node["resolution"] = rhs.resolution;
	node["data"] = compressHex(rhs.data);

	return node;
}

bool convert<octomap_msgs::Octomap>::decode(const Node& node, octomap_msgs::Octomap& rhs) {
	rhs = octomap_msgs::Octomap();

	if (node["header"])
		rhs.header = node["header"].as<std_msgs::Header>();
	else
		rhs.header = getDefaultHeader();

	if (node["binary"])
		rhs.binary = nodeToBool(node["binary"]);

	if (node["id"])
		rhs.id = node["id"].as<std::string>();

	if (node["resolution"])
		rhs.resolution = node["resolution"].as<double>();

	if (node["data"]) {
		// Load old octomap formats / direct YAML output
		if (node["data"].IsSequence()) {
			auto temp = node["data"].as<std::vector<int>>();
			rhs.data = std::vector<int8_t>(temp.begin(), temp.end());
		} else {
			auto temp = node["data"].as<std::string>();
			rhs.data = decompressHex(temp);
		}
	}

	return true;
}

Node convert<octomap_msgs::OctomapWithPose>::encode(const octomap_msgs::OctomapWithPose& rhs) {
	Node node;

	if (!isHeaderEmpty(rhs.header))
		node["header"] = rhs.header;

	node["origin"] = rhs.origin;
	node["octomap"] = rhs.octomap;
	return node;
}

bool convert<octomap_msgs::OctomapWithPose>::decode(const Node& node, octomap_msgs::OctomapWithPose& rhs) {
	rhs = octomap_msgs::OctomapWithPose();

	if (node["header"])
		rhs.header = node["header"].as<std_msgs::Header>();
	else
		rhs.header = getDefaultHeader();

	if (node["origin"])
		rhs.origin = node["origin"].as<geometry_msgs::Pose>();

	if (node["octomap"])
		rhs.octomap = node["octomap"].as<octomap_msgs::Octomap>();

	return true;
}

}  // namespace YAML
