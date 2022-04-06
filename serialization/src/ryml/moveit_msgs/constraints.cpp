#include <moveit_serialization/ryml/std.h>
#include <moveit_serialization/ryml/moveit_msgs/joint_constraint.h>
#include <moveit_serialization/ryml/moveit_msgs/position_constraint.h>
#include <moveit_serialization/ryml/moveit_msgs/orientation_constraint.h>
#include <moveit_serialization/ryml/moveit_msgs/visibility_constraint.h>

#include <moveit_serialization/ryml/moveit_msgs/constraints.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::Constraints const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("name") << rhs.name;
    n->append_child() << yml::key("joint_constraints") << rhs.joint_constraints;
    n->append_child() << yml::key("position_constraints") << rhs.position_constraints;
    n->append_child() << yml::key("orientation_constraints") << rhs.orientation_constraints;
    n->append_child() << yml::key("visibility_constraints") << rhs.visibility_constraints;
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::Constraints* rhs)
{
    if (n.has_child("name"))
        n["name"] >> rhs->name;
    if (n.has_child("joint_constraints"))
        n["joint_constraints"] >> rhs->joint_constraints;
    if (n.has_child("position_constraints"))
        n["position_constraints"] >> rhs->position_constraints;
    if (n.has_child("orientation_constraints"))
        n["orientation_constraints"] >> rhs->orientation_constraints;
    if (n.has_child("visibility_constraints"))
        n["visibility_constraints"] >> rhs->visibility_constraints;

    return true;
}

}  // namespace yml
}  // namespace c4
