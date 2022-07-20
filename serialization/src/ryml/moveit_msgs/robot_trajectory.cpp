#include <moveit_serialization/ryml/trajectory_msgs/joint_trajectory.h>
#include <moveit_serialization/ryml/trajectory_msgs/multidof_joint_trajectory.h>

#include <moveit_serialization/ryml/moveit_msgs/robot_trajectory.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::RobotTrajectory const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("joint_trajectory") << rhs.joint_trajectory;
    n->append_child() << yml::key("multi_dof_joint_trajectory") << rhs.multi_dof_joint_trajectory;
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::RobotTrajectory* rhs)
{
    if (n.has_child("joint_trajectory"))
        n["joint_trajectory"] >> rhs->joint_trajectory;
    if (n.has_child("multi_dof_joint_trajectory"))
        n["multi_dof_joint_trajectory"] >> rhs->multi_dof_joint_trajectory;

    return true;
}

}  // namespace yml
}  // namespace c4
