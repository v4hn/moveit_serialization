#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/format.h>
#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/geometry_msgs/vector3.h>
#include <moveit_serialization/ryml/moveit_msgs/bounding_volume.h>

#include <moveit_serialization/ryml/moveit_msgs/position_constraint.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::PositionConstraint const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("link_name") << rhs.link_name;
    n->append_child() << yml::key("target_point_offset") << rhs.target_point_offset;
    n->append_child() << yml::key("constraint_region") << rhs.constraint_region;
    n->append_child() << yml::key("weight") << freal(rhs.weight);
}

bool read(c4::yml::ConstNodeRef const& n, moveit_msgs::PositionConstraint* rhs)
{
    if (n.has_child("header"))
        n["header"] >> rhs->header;
    if (n.has_child("link_name"))
        n["link_name"] >> rhs->link_name;
    if (n.has_child("target_point_offset"))
        n["target_point_offset"] >> rhs->target_point_offset;
    if (n.has_child("constraint_region"))
        n["constraint_region"] >> rhs->constraint_region;
    if (n.has_child("weight"))
        n["weight"] >> rhs->weight;

    return true;
}

}  // namespace yml
}  // namespace c4
