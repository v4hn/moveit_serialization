#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/format.h>

#include <moveit_serialization/ryml/moveit_msgs/link_scale.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::LinkScale const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("link_name") << rhs.link_name;
    n->append_child() << yml::key("scale") << freal(rhs.scale);
}

bool read(c4::yml::ConstNodeRef const& n, moveit_msgs::LinkScale* rhs)
{
    if (n.has_child("link_name"))
        n["link_name"] >> rhs->link_name;
    if (n.has_child("scale"))
        n["scale"] >> rhs->scale;

    return true;
}

}  // namespace yml
}  // namespace c4
