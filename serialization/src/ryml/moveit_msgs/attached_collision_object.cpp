#include <moveit_serialization/ryml/std.h>
#include <moveit_serialization/ryml/moveit_msgs/collision_object.h>
#include <moveit_serialization/ryml/trajectory_msgs/joint_trajectory.h>

#include <moveit_serialization/ryml/moveit_msgs/attached_collision_object.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::AttachedCollisionObject const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("link_name") << rhs.link_name;
    n->append_child() << yml::key("object") << rhs.object;
    n->append_child() << yml::key("touch_links") << rhs.touch_links;
    n->append_child() << yml::key("detach_posture") << rhs.detach_posture;
    n->append_child() << yml::key("weight") << freal(rhs.weight);
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::AttachedCollisionObject* rhs)
{
    if (n.has_child("link_name"))
        n["link_name"] >> rhs->link_name;
    if (n.has_child("object"))
        n["object"] >> rhs->object;
    if (n.has_child("touch_links"))
        n["touch_links"] >> rhs->touch_links;
    if (n.has_child("detach_posture"))
        n["detach_posture"] >> rhs->detach_posture;
    if (n.has_child("weight"))
        n["weight"] >> rhs->weight;

    return true;
}

}  // namespace yml
}  // namespace c4
