#include <moveit_serialization/yaml-cpp/conversion/sensor_msgs.h>

#include <moveit_serialization/yaml-cpp/conversion/std_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/geometry_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/utilities.h>

namespace YAML {
Node convert<sensor_msgs::JointState>::encode(const sensor_msgs::JointState& rhs)
{
    Node node;

    if (!isHeaderEmpty(rhs.header))
        node["header"] = rhs.header;

    if (!rhs.name.empty()) {
        node["name"] = rhs.name;
        node["name"].SetStyle(EmitterStyle::Flow);
    }

    if (!rhs.position.empty()) {
        node["position"] = rhs.position;
        node["position"].SetStyle(EmitterStyle::Flow);
    }

    if (!rhs.velocity.empty()) {
        node["velocity"] = rhs.velocity;
        node["velocity"].SetStyle(EmitterStyle::Flow);
    }

    if (!rhs.effort.empty()) {
        node["effort"] = rhs.effort;
        node["effort"].SetStyle(EmitterStyle::Flow);
    }

    return node;
}

bool convert<sensor_msgs::JointState>::decode(const Node& node, sensor_msgs::JointState& rhs)
{
    rhs = sensor_msgs::JointState();

    if (node["header"])
        rhs.header = node["header"].as<std_msgs::Header>();
    else
        rhs.header = getDefaultHeader();

    if (node["name"])
        rhs.name = node["name"].as<std::vector<std::string>>();

    if (node["position"])
        rhs.position = node["position"].as<std::vector<double>>();

    if (node["velocity"])
        rhs.velocity = node["velocity"].as<std::vector<double>>();

    if (node["effort"])
        rhs.effort = node["effort"].as<std::vector<double>>();

    return true;
}

Node convert<sensor_msgs::MultiDOFJointState>::encode(const sensor_msgs::MultiDOFJointState& rhs)
{
    Node node;

    if (!isHeaderEmpty(rhs.header))
        node["header"] = rhs.header;

    node["joint_names"] = rhs.joint_names;
    node["joint_names"].SetStyle(EmitterStyle::Flow);

    node["transforms"] = rhs.transforms;
    node["transforms"].SetStyle(EmitterStyle::Flow);

    node["twist"] = rhs.twist;
    node["twist"].SetStyle(EmitterStyle::Flow);

    node["wrench"] = rhs.wrench;
    node["wrench"].SetStyle(EmitterStyle::Flow);
    return node;
}

bool convert<sensor_msgs::MultiDOFJointState>::decode(const Node& node, sensor_msgs::MultiDOFJointState& rhs)
{
    rhs = sensor_msgs::MultiDOFJointState();

    if (node["header"])
        rhs.header = node["header"].as<std_msgs::Header>();
    else
        rhs.header = getDefaultHeader();

    if (node["joint_names"])
        rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();

    if (node["transforms"])
        rhs.transforms = node["transforms"].as<std::vector<geometry_msgs::Transform>>();

    if (node["twist"])
        rhs.twist = node["twist"].as<std::vector<geometry_msgs::Twist>>();

    if (node["wrench"])
        rhs.wrench = node["wrench"].as<std::vector<geometry_msgs::Wrench>>();

    return true;
}

}  // namespace YAML
