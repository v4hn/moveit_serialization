#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/std_msgs/color_rgba.h>

#include <moveit_serialization/ryml/moveit_msgs/object_color.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::ObjectColor const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("id") << rhs.id;
    n->append_child() << yml::key("color") << rhs.color;
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::ObjectColor* rhs)
{
    if (n.has_child("id"))
        n["id"] >> rhs->id;
    if (n.has_child("color"))
        n["color"] >> rhs->color;

    return true;
}

}  // namespace yml
}  // namespace c4
