#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/moveit_msgs/constraints.h>

#include <moveit_serialization/ryml/moveit_msgs/trajectory_constraints.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::TrajectoryConstraints const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("constraints") << rhs.constraints;
}

bool read(c4::yml::ConstNodeRef const& n, moveit_msgs::TrajectoryConstraints* rhs)
{
    if (n.has_child("constraints"))
        n["constraints"] >> rhs->constraints;

    return true;
}

}  // namespace yml
}  // namespace c4
