#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/format.h>

#include <moveit_serialization/ryml/moveit_msgs/link_padding.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::LinkPadding const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("link_name") << rhs.link_name;
    n->append_child() << yml::key("padding") << freal(rhs.padding);
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::LinkPadding* rhs)
{
    if (n.has_child("link_name"))
        n["link_name"] >> rhs->link_name;
    if (n.has_child("padding"))
        n["padding"] >> rhs->padding;

    return true;
}

}  // namespace yml
}  // namespace c4
