#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/moveit_msgs/allowed_collision_entry.h>

#include <moveit_serialization/ryml/moveit_msgs/allowed_collision_matrix.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::AllowedCollisionMatrix const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("entry_names") << rhs.entry_names;
    n->append_child() << yml::key("entry_values") << rhs.entry_values;
    n->append_child() << yml::key("default_entry_names") << rhs.default_entry_names;
    n->append_child() << yml::key("default_entry_values") << rhs.default_entry_values;
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::AllowedCollisionMatrix* rhs)
{
    if (n.has_child("entry_names"))
        n["entry_names"] >> rhs->entry_names;
    if (n.has_child("entry_values"))
        n["entry_values"] >> rhs->entry_values;
    if (n.has_child("default_entry_names"))
        n["default_entry_names"] >> rhs->default_entry_names;
    if (n.has_child("default_entry_values"))
        n["default_entry_values"] >> rhs->default_entry_values;

    return true;
}

}  // namespace yml
}  // namespace c4
