#include <moveit_serialization/ryml/ros/duration.h>
#include <moveit_serialization/ryml/moveit_msgs/cartesian_point.h>

#include <moveit_serialization/ryml/moveit_msgs/cartesian_trajectory_point.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::CartesianTrajectoryPoint const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("point") << rhs.point;
    n->append_child() << yml::key("time_from_start") << rhs.time_from_start;
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::CartesianTrajectoryPoint* rhs)
{
    if (n.has_child("point"))
        n["point"] >> rhs->point;
    if (n.has_child("time_from_start"))
        n["time_from_start"] >> rhs->time_from_start;

    return true;
}

}  // namespace yml
}  // namespace c4
