#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/geometry_msgs/pose.h>
#include <moveit_serialization/ryml/octomap_msgs/octomap.h>

#include <moveit_serialization/ryml/octomap_msgs/octomap_with_pose.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, octomap_msgs::OctomapWithPose const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("origin") << rhs.origin;
    n->append_child() << yml::key("octomap") << rhs.octomap;
}

bool read(c4::yml::ConstNodeRef const& n, octomap_msgs::OctomapWithPose* rhs)
{
    if (n.has_child("header"))
        n["header"] >> rhs->header;
    if (n.has_child("origin"))
        n["origin"] >> rhs->origin;
    if (n.has_child("octomap"))
        n["octomap"] >> rhs->octomap;

    return true;
}

}  // namespace yml
}  // namespace c4
