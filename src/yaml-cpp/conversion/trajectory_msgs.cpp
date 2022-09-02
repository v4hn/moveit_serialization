#include <moveit_serialization/yaml-cpp/conversion/trajectory_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/geometry_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/std_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/ros.h>
#include <moveit_serialization/yaml-cpp/conversion/utilities.h>

namespace YAML {

Node convert<trajectory_msgs::JointTrajectory>::encode(const trajectory_msgs::JointTrajectory& rhs)
{
    Node node;

    if (!isHeaderEmpty(rhs.header))
        node["header"] = rhs.header;

    node["joint_names"] = rhs.joint_names;
    node["joint_names"].SetStyle(EmitterStyle::Flow);
    node["points"] = rhs.points;
    return node;
}

bool convert<trajectory_msgs::JointTrajectory>::decode(const Node& node, trajectory_msgs::JointTrajectory& rhs)
{
    rhs = trajectory_msgs::JointTrajectory();

    if (node["header"])
        rhs.header = node["header"].as<std_msgs::Header>();
    else
        rhs.header = getDefaultHeader();

    if (node["joint_names"])
        rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();

    if (node["points"])
        rhs.points = node["points"].as<std::vector<trajectory_msgs::JointTrajectoryPoint>>();

    return true;
}

Node convert<trajectory_msgs::JointTrajectoryPoint>::encode(const trajectory_msgs::JointTrajectoryPoint& rhs)
{
    Node node;

    if (!rhs.positions.empty()) {
        node["positions"] = rhs.positions;
        node["positions"].SetStyle(EmitterStyle::Flow);
    }

    if (!rhs.velocities.empty()) {
        node["velocities"] = rhs.velocities;
        node["velocities"].SetStyle(EmitterStyle::Flow);
    }

    if (!rhs.accelerations.empty()) {
        node["accelerations"] = rhs.accelerations;
        node["accelerations"].SetStyle(EmitterStyle::Flow);
    }

    if (!rhs.effort.empty()) {
        node["effort"] = rhs.effort;
        node["effort"].SetStyle(EmitterStyle::Flow);
    }

    node["time_from_start"] = rhs.time_from_start;

    return node;
}

bool convert<trajectory_msgs::JointTrajectoryPoint>::decode(const Node& node, trajectory_msgs::JointTrajectoryPoint& rhs)
{
    if (node["positions"])
        rhs.positions = node["positions"].as<std::vector<double>>();

    if (node["velocities"])
        rhs.velocities = node["velocities"].as<std::vector<double>>();

    if (node["accelerations"])
        rhs.accelerations = node["accelerations"].as<std::vector<double>>();

    if (node["effort"])
        rhs.effort = node["effort"].as<std::vector<double>>();

    if (node["time_from_start"])
        rhs.time_from_start = node["time_from_start"].as<ros::Duration>();

    return true;
}

Node convert<trajectory_msgs::MultiDOFJointTrajectory>::encode(const trajectory_msgs::MultiDOFJointTrajectory& rhs)
{
    Node node;

    if (!isHeaderEmpty(rhs.header))
        node["header"] = rhs.header;

    node["joint_names"] = rhs.joint_names;
    node["points"] = rhs.points;

    return node;
}

bool convert<trajectory_msgs::MultiDOFJointTrajectory>::decode(const Node& node,
                                                               trajectory_msgs::MultiDOFJointTrajectory& rhs)
{
    rhs = trajectory_msgs::MultiDOFJointTrajectory();

    if (node["header"])
        rhs.header = node["header"].as<std_msgs::Header>();

    if (node["joint_names"])
        rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();

    if (node["points"])
        rhs.points = node["points"].as<std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>>();

    return true;
}

Node convert<trajectory_msgs::MultiDOFJointTrajectoryPoint>::encode(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& rhs)
{
    Node node;

    if (!rhs.transforms.empty())
        node["transforms"] = rhs.transforms;

    if (!rhs.velocities.empty())
        node["velocities"] = rhs.velocities;

    if (!rhs.accelerations.empty())
        node["accelerations"] = rhs.accelerations;

    node["time_from_start"] = rhs.time_from_start;

    return node;
}

bool convert<trajectory_msgs::MultiDOFJointTrajectoryPoint>::decode(const Node& node,
                                                                    trajectory_msgs::MultiDOFJointTrajectoryPoint& rhs)
{
    rhs = trajectory_msgs::MultiDOFJointTrajectoryPoint();

    if (node["transforms"])
        rhs.transforms = node["transforms"].as<std::vector<geometry_msgs::Transform>>();

    if (node["velocities"])
        rhs.velocities = node["velocities"].as<std::vector<geometry_msgs::Twist>>();

    if (node["accelerations"])
        rhs.accelerations = node["accelerations"].as<std::vector<geometry_msgs::Twist>>();

    rhs.time_from_start = node["time_from_start"].as<ros::Duration>();
    return true;
}

}  // namespace YAML
