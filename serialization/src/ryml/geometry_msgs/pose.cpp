#include <moveit_serialization/ryml/geometry_msgs/point.h>
#include <moveit_serialization/ryml/geometry_msgs/quaternion.h>
#include <moveit_serialization/ryml/geometry_msgs/pose.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, geometry_msgs::Pose const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("position") << rhs.position;
    n->append_child() << yml::key("orientation") << rhs.orientation;
}

bool read(c4::yml::NodeRef const& n, geometry_msgs::Pose* rhs)
{
    n["position"] >> rhs->position;
    n["orientation"] >> rhs->orientation;

    return true;
}

}  // namespace yml
}  // namespace c4
