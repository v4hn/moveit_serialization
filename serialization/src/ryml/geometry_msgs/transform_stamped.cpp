#include <moveit_serialization/ryml/std.h>

#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/geometry_msgs/transform.h>
#include <moveit_serialization/ryml/geometry_msgs/transform_stamped.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, geometry_msgs::TransformStamped const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("child_frame_id") << rhs.child_frame_id;
    n->append_child() << yml::key("transform") << rhs.transform;
}

bool read(c4::yml::NodeRef const& n, geometry_msgs::TransformStamped* rhs)
{
    n["header"] >> rhs->header;
    n["child_frame_id"] >> rhs->child_frame_id;
    n["transform"] >> rhs->transform;

    return true;
}

}  // namespace yml
}  // namespace c4
