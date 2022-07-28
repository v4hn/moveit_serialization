#include <moveit_serialization/ryml/std/std.h>

#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/geometry_msgs/transform.h>
#include <moveit_serialization/ryml/geometry_msgs/twist.h>
#include <moveit_serialization/ryml/geometry_msgs/wrench.h>
#include <moveit_serialization/ryml/sensor_msgs/multidof_joint_state.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, sensor_msgs::MultiDOFJointState const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("joint_names") << rhs.joint_names |= yml::_WIP_STYLE_FLOW_SL;
    n->append_child() << yml::key("transforms") << rhs.transforms |= yml::_WIP_STYLE_FLOW_SL;
    n->append_child() << yml::key("twist") << rhs.twist |= yml::_WIP_STYLE_FLOW_SL;
    n->append_child() << yml::key("wrench") << rhs.wrench |= yml::_WIP_STYLE_FLOW_SL;
}

bool read(c4::yml::NodeRef const& n, sensor_msgs::MultiDOFJointState* rhs)
{
    n["header"] >> rhs->header;
    n["joint_names"] >> rhs->joint_names;

    if (n.has_child("transforms"))
        n["transforms"] >> rhs->transforms;
    if (n.has_child("twist"))
        n["twist"] >> rhs->twist;
    if (n.has_child("wrench"))
        n["wrench"] >> rhs->wrench;

    return true;
}

}  // namespace yml
}  // namespace c4
