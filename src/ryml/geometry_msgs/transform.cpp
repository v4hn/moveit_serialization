#include <moveit_serialization/ryml/geometry_msgs/vector3.h>
#include <moveit_serialization/ryml/geometry_msgs/quaternion.h>
#include <moveit_serialization/ryml/geometry_msgs/transform.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, geometry_msgs::Transform const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("translation") << rhs.translation;
    n->append_child() << yml::key("rotation") << rhs.rotation;
}

bool read(c4::yml::ConstNodeRef const& n, geometry_msgs::Transform* rhs)
{
    n["translation"] >> rhs->translation;
    n["rotation"] >> rhs->rotation;

    return true;
}

}  // namespace yml
}  // namespace c4
