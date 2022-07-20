#include <moveit_serialization/ryml/std.h>
#include <moveit_serialization/ryml/format.h>

#include <moveit_serialization/ryml/moveit_msgs/joint_constraint.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::JointConstraint const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("joint_name") << rhs.joint_name;
    n->append_child() << yml::key("position") << freal(rhs.position);
    n->append_child() << yml::key("tolerance_above") << freal(rhs.tolerance_above);
    n->append_child() << yml::key("tolerance_below") << freal(rhs.tolerance_below);
    n->append_child() << yml::key("weight") << freal(rhs.weight);
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::JointConstraint* rhs)
{
    if (n.has_child("joint_name"))
        n["joint_name"] >> rhs->joint_name;
    if (n.has_child("position"))
        n["position"] >> rhs->position;
    if (n.has_child("tolerance_above"))
        n["tolerance_above"] >> rhs->tolerance_above;
    if (n.has_child("tolerance_below"))
        n["tolerance_below"] >> rhs->tolerance_below;
    if (n.has_child("weight"))
        n["weight"] >> rhs->weight;

    return true;
}

}  // namespace yml
}  // namespace c4
