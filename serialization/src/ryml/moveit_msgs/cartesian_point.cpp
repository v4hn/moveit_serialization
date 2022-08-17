#include <moveit_serialization/ryml/geometry_msgs/pose.h>
#include <moveit_serialization/ryml/geometry_msgs/twist.h>
#include <moveit_serialization/ryml/geometry_msgs/accel.h>

#include <moveit_serialization/ryml/moveit_msgs/cartesian_point.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::CartesianPoint const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("pose") << rhs.pose;
    n->append_child() << yml::key("velocity") << rhs.velocity;
    n->append_child() << yml::key("acceleration") << rhs.acceleration;
}

bool read(c4::yml::ConstNodeRef const& n, moveit_msgs::CartesianPoint* rhs)
{
    if (n.has_child("pose"))
        n["pose"] >> rhs->pose;
    if (n.has_child("velocity"))
        n["velocity"] >> rhs->velocity;
    if (n.has_child("acceleration"))
        n["acceleration"] >> rhs->acceleration;

    return true;
}

}  // namespace yml
}  // namespace c4
