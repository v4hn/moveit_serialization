#include <moveit_serialization/ryml/geometry_msgs/vector3.h>
#include <moveit_serialization/ryml/geometry_msgs/wrench.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, geometry_msgs::Wrench const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("force") << rhs.force;
    n->append_child() << yml::key("torque") << rhs.torque;
}

bool read(c4::yml::ConstNodeRef const& n, geometry_msgs::Wrench* rhs)
{
    n["force"] >> rhs->force;
    n["torque"] >> rhs->torque;

    return true;
}

}  // namespace yml
}  // namespace c4
