#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/geometry_msgs/point.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, geometry_msgs::Point const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("x") << freal(rhs.x);
    n->append_child() << yml::key("y") << freal(rhs.y);
    n->append_child() << yml::key("z") << freal(rhs.z);
}

bool read(c4::yml::ConstNodeRef const& n, geometry_msgs::Point* rhs)
{
    n["x"] >> rhs->x;
    n["y"] >> rhs->y;
    n["z"] >> rhs->z;

    return true;
}

}  // namespace yml
}  // namespace c4
