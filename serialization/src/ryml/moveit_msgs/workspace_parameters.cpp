#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/geometry_msgs/vector3.h>

#include <moveit_serialization/ryml/moveit_msgs/workspace_parameters.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::WorkspaceParameters const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("min_corner") << rhs.min_corner;
    n->append_child() << yml::key("max_corner") << rhs.max_corner;
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::WorkspaceParameters* rhs)
{
    if (n.has_child("header"))
        n["header"] >> rhs->header;
    if (n.has_child("min_corner"))
        n["min_corner"] >> rhs->min_corner;
    if (n.has_child("max_corner"))
        n["max_corner"] >> rhs->max_corner;

    return true;
}

}  // namespace yml
}  // namespace c4
