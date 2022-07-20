#include <moveit_serialization/ryml/std.h>

#include <moveit_serialization/ryml/moveit_msgs/allowed_collision_entry.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::AllowedCollisionEntry const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("enabled") << rhs.enabled;
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::AllowedCollisionEntry* rhs)
{
    if (n.has_child("enabled"))
        n["enabled"] >> rhs->enabled;

    return true;
}

}  // namespace yml
}  // namespace c4
