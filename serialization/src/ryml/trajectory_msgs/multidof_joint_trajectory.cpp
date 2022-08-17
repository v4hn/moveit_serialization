#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/trajectory_msgs/multidof_joint_trajectory_point.h>

#include <moveit_serialization/ryml/trajectory_msgs/multidof_joint_trajectory.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, trajectory_msgs::MultiDOFJointTrajectory const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("joint_names") << rhs.joint_names;
    n->append_child() << yml::key("points") << rhs.points;
}

bool read(c4::yml::ConstNodeRef const& n, trajectory_msgs::MultiDOFJointTrajectory* rhs)
{
    if (n.has_child("header"))
        n["header"] >> rhs->header;
    if (n.has_child("joint_names"))
        n["joint_names"] >> rhs->joint_names;
    if (n.has_child("points"))
        n["points"] >> rhs->points;

    return true;
}

}  // namespace yml
}  // namespace c4
