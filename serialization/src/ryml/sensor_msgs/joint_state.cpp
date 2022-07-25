#include <moveit_serialization/ryml/std/std.h>

#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/sensor_msgs/joint_state.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, sensor_msgs::JointState const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("name") << rhs.name;
    n->append_child() << yml::key("position") << rhs.position;
    n->append_child() << yml::key("velocity") << rhs.velocity;
    n->append_child() << yml::key("effort") << rhs.effort;
}

bool read(c4::yml::NodeRef const& n, sensor_msgs::JointState* rhs)
{
    n["header"] >> rhs->header;
    n["name"] >> rhs->name;

    if (n.has_child("position"))
        n["position"] >> rhs->position;
    if (n.has_child("velocity"))
        n["velocity"] >> rhs->velocity;
    if (n.has_child("effort"))
        n["effort"] >> rhs->effort;

    return true;
}

}  // namespace yml
}  // namespace c4
