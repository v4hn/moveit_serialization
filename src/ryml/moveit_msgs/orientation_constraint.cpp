#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/format.h>
#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/geometry_msgs/quaternion.h>

#include <moveit_serialization/ryml/moveit_msgs/orientation_constraint.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::OrientationConstraint const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("orientation") << rhs.orientation;
    n->append_child() << yml::key("link_name") << rhs.link_name;
    n->append_child() << yml::key("absolute_x_axis_tolerance") << freal(rhs.absolute_x_axis_tolerance);
    n->append_child() << yml::key("absolute_y_axis_tolerance") << freal(rhs.absolute_y_axis_tolerance);
    n->append_child() << yml::key("absolute_z_axis_tolerance") << freal(rhs.absolute_z_axis_tolerance);
    n->append_child() << yml::key("parameterization") << rhs.parameterization;
    n->append_child() << yml::key("weight") << freal(rhs.weight);
}

bool read(c4::yml::ConstNodeRef const& n, moveit_msgs::OrientationConstraint* rhs)
{
    if (n.has_child("header"))
        n["header"] >> rhs->header;
    if (n.has_child("orientation"))
        n["orientation"] >> rhs->orientation;
    if (n.has_child("link_name"))
        n["link_name"] >> rhs->link_name;
    if (n.has_child("absolute_x_axis_tolerance"))
        n["absolute_x_axis_tolerance"] >> rhs->absolute_x_axis_tolerance;
    if (n.has_child("absolute_y_axis_tolerance"))
        n["absolute_y_axis_tolerance"] >> rhs->absolute_y_axis_tolerance;
    if (n.has_child("absolute_z_axis_tolerance"))
        n["absolute_z_axis_tolerance"] >> rhs->absolute_z_axis_tolerance;
    if (n.has_child("parameterization"))
        n["parameterization"] >> rhs->parameterization;
    if (n.has_child("weight"))
        n["weight"] >> rhs->weight;

    return true;
}

}  // namespace yml
}  // namespace c4
