

#include <boost/algorithm/hex.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <moveit_serialization/yaml-cpp/conversion/utilities.h>

#include <moveit_msgs/CollisionObject.h>

namespace YAML {

// Many ROS msgs uses the primitive type uint8 which translate to uint8_t in c++. The
// yaml-cpp library converts them to un/signed char which. HACK encode/decode as uint16_t.
void encodeToUINT8(YAML::Node& n, const std::string& key, const uint8_t& data) {
	n[key] = static_cast<uint16_t>(data);
}

void decodeToUINT8(const YAML::Node& n, const std::string& key, uint8_t& data) {
	data = n[key].as<uint16_t>();
}

std::string boolToString(bool b) {
	return b ? "true" : "false";
}

bool nodeToBool(const YAML::Node& n) {
	std::string s = n.as<std::string>();
	std::transform(s.begin(), s.end(), s.begin(), ::tolower);
	return s == "true";
}

bool isHeaderEmpty(const std_msgs::Header& h) {
	return h.seq == 0 && h.stamp.isZero() && h.frame_id == "world";
}

std_msgs::Header getDefaultHeader() {
	std_msgs::Header msg;
	msg.frame_id = "world";
	return msg;
}
unsigned int nodeToCollisionObject(const YAML::Node& n) {
	try {
		std::string s = n.as<std::string>();
		std::transform(s.begin(), s.end(), s.begin(), ::tolower);

		if (s == "move")
			return moveit_msgs::CollisionObject::MOVE;
		if (s == "remove")
			return moveit_msgs::CollisionObject::REMOVE;
		if (s == "append")
			return moveit_msgs::CollisionObject::APPEND;

		return moveit_msgs::CollisionObject::ADD;
	} catch (const YAML::BadConversion& e) {
		// Sometimes it is specified as the int.
		int op = n.as<int>();
		switch (op) {
			case 0:
				return moveit_msgs::CollisionObject::ADD;
			case 1:
				return moveit_msgs::CollisionObject::REMOVE;
			case 2:
				return moveit_msgs::CollisionObject::APPEND;
			case 3:
				return moveit_msgs::CollisionObject::MOVE;
			default:
				return moveit_msgs::CollisionObject::ADD;
		}
	}
}

bool isVector3Zero(const geometry_msgs::Vector3& v) {
	return v.x == 0 && v.y == 0 && v.z == 0;
}

bool isConstraintEmpty(const moveit_msgs::Constraints& c) {
	return c.joint_constraints.empty()  //
	       && c.position_constraints.empty()  //
	       && c.orientation_constraints.empty()  //
	       && c.visibility_constraints.empty();
}

std::string compressHex(const std::vector<int8_t>& v) {
	std::vector<char> compress;
	{
		boost::iostreams::filtering_ostream fos;
		fos.push(boost::iostreams::zlib_compressor());
		fos.push(boost::iostreams::back_inserter(compress));

		for (const auto& i : v)
			fos << i;
	}

	std::string result;
	boost::algorithm::hex(compress.begin(), compress.end(), std::back_inserter(result));

	return result;
}

std::vector<int8_t> decompressHex(const std::string& hex) {
	std::vector<int8_t> unhexed;
	boost::algorithm::unhex(hex, std::back_inserter(unhexed));

	std::vector<char> decompress;
	{
		boost::iostreams::filtering_ostream fos;
		fos.push(boost::iostreams::zlib_decompressor());
		fos.push(boost::iostreams::back_inserter(decompress));

		for (const auto& i : unhexed)
			fos << i;
	}

	return std::vector<int8_t>(decompress.begin(), decompress.end());
}

}  // namespace YAML
