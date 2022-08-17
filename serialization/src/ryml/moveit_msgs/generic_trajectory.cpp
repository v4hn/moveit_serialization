#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/trajectory_msgs/joint_trajectory.h>
#include <moveit_serialization/ryml/moveit_msgs/cartesian_trajectory.h>

#include <moveit_serialization/ryml/moveit_msgs/generic_trajectory.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::GenericTrajectory const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("joint_trajectory") << rhs.joint_trajectory;
    n->append_child() << yml::key("cartesian_trajectory") << rhs.cartesian_trajectory;
}

bool read(c4::yml::ConstNodeRef const& n, moveit_msgs::GenericTrajectory* rhs)
{
    if (n.has_child("header"))
        n["header"] >> rhs->header;
    if (n.has_child("joint_trajectory"))
        n["joint_trajectory"] >> rhs->joint_trajectory;
    if (n.has_child("cartesian_trajectory"))
        n["cartesian_trajectory"] >> rhs->cartesian_trajectory;

    return true;
}

}  // namespace yml
}  // namespace c4
