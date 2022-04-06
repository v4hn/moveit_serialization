#include <moveit_serialization/ryml/std.h>
#include <moveit_serialization/ryml/octomap_msgs/octomap_with_pose.h>
#include <moveit_serialization/ryml/moveit_msgs/collision_object.h>

#include <moveit_serialization/ryml/moveit_msgs/planning_scene_world.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::PlanningSceneWorld const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("collision_objects") << rhs.collision_objects;
    n->append_child() << yml::key("octomap") << rhs.octomap;
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::PlanningSceneWorld* rhs)
{
    if (n.has_child("collision_objects"))
        n["collision_objects"] >> rhs->collision_objects;
    if (n.has_child("octomap"))
        n["octomap"] >> rhs->octomap;

    return true;
}

}  // namespace yml
}  // namespace c4
