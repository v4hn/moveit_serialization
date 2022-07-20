#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/geometry_msgs/pose.h>
#include <moveit_serialization/ryml/geometry_msgs/pose_stamped.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, geometry_msgs::PoseStamped const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("pose") << rhs.pose;
}

bool read(c4::yml::NodeRef const& n, geometry_msgs::PoseStamped* rhs)
{
    n["header"] >> rhs->header;
    n["pose"] >> rhs->pose;

    return true;
}

}  // namespace yml
}  // namespace c4
