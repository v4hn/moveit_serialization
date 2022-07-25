#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/moveit_msgs/cartesian_trajectory_point.h>

#include <moveit_serialization/ryml/moveit_msgs/cartesian_trajectory.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::CartesianTrajectory const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("tracked_frame") << rhs.tracked_frame;
    n->append_child() << yml::key("points") << rhs.points;
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::CartesianTrajectory* rhs)
{
    if (n.has_child("header"))
        n["header"] >> rhs->header;
    if (n.has_child("tracked_frame"))
        n["tracked_frame"] >> rhs->tracked_frame;
    if (n.has_child("points"))
        n["points"] >> rhs->points;

    return true;
}

}  // namespace yml
}  // namespace c4
