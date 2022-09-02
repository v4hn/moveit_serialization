#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/geometry_msgs/transform.h>
#include <moveit_serialization/ryml/geometry_msgs/twist.h>
#include <moveit_serialization/ryml/ros/duration.h>

#include <moveit_serialization/ryml/trajectory_msgs/multidof_joint_trajectory_point.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, trajectory_msgs::MultiDOFJointTrajectoryPoint const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("transforms") << rhs.transforms;
    n->append_child() << yml::key("velocities") << rhs.velocities;
    n->append_child() << yml::key("accelerations") << rhs.accelerations;
    n->append_child() << yml::key("time_from_start") << rhs.time_from_start;
}

bool read(c4::yml::ConstNodeRef const& n, trajectory_msgs::MultiDOFJointTrajectoryPoint* rhs)
{
    if (n.has_child("transforms"))
        n["transforms"] >> rhs->transforms;
    if (n.has_child("velocities"))
        n["velocities"] >> rhs->velocities;
    if (n.has_child("accelerations"))
        n["accelerations"] >> rhs->accelerations;
    if (n.has_child("time_from_start"))
        n["time_from_start"] >> rhs->time_from_start;

    return true;
}

}  // namespace yml
}  // namespace c4
