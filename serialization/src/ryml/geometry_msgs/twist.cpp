#include <moveit_serialization/ryml/geometry_msgs/vector3.h>
#include <moveit_serialization/ryml/geometry_msgs/twist.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, geometry_msgs::Twist const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("linear") << rhs.linear;
    n->append_child() << yml::key("angular") << rhs.angular;
}

bool read(c4::yml::NodeRef const& n, geometry_msgs::Twist* rhs)
{
    n["linear"] >> rhs->linear;
    n["angular"] >> rhs->angular;

    return true;
}

}  // namespace yml
}  // namespace c4
