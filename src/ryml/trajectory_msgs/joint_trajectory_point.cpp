#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/ros/duration.h>

#include <moveit_serialization/ryml/trajectory_msgs/joint_trajectory_point.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, trajectory_msgs::JointTrajectoryPoint const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("positions") << rhs.positions |= yml::_WIP_STYLE_FLOW_SL;
    n->append_child() << yml::key("velocities") << rhs.velocities |= yml::_WIP_STYLE_FLOW_SL;
    n->append_child() << yml::key("accelerations") << rhs.accelerations |= yml::_WIP_STYLE_FLOW_SL;
    n->append_child() << yml::key("effort") << rhs.effort |= yml::_WIP_STYLE_FLOW_SL;
    n->append_child() << yml::key("time_from_start") << rhs.time_from_start;
}

bool read(c4::yml::ConstNodeRef const& n, trajectory_msgs::JointTrajectoryPoint* rhs)
{
    if (n.has_child("positions"))
        n["positions"] >> rhs->positions;
    if (n.has_child("velocities"))
        n["velocities"] >> rhs->velocities;
    if (n.has_child("accelerations"))
        n["accelerations"] >> rhs->accelerations;
    if (n.has_child("effort"))
        n["effort"] >> rhs->effort;
    if (n.has_child("time_from_start"))
        n["time_from_start"] >> rhs->time_from_start;

    return true;
}

}  // namespace yml
}  // namespace c4
