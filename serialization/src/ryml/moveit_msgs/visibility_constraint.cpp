#include <moveit_serialization/ryml/format.h>
#include <moveit_serialization/ryml/geometry_msgs/pose_stamped.h>

#include <moveit_serialization/ryml/moveit_msgs/visibility_constraint.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::VisibilityConstraint const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("target_radius") << freal(rhs.target_radius);
    n->append_child() << yml::key("target_pose") << rhs.target_pose;
    n->append_child() << yml::key("cone_sides") << rhs.cone_sides;
    n->append_child() << yml::key("sensor_pose") << rhs.sensor_pose;
    n->append_child() << yml::key("max_view_angle") << freal(rhs.max_view_angle);
    n->append_child() << yml::key("max_range_angle") << freal(rhs.max_range_angle);
    n->append_child() << yml::key("sensor_view_direction") << rhs.sensor_view_direction;
    n->append_child() << yml::key("weight") << freal(rhs.weight);
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::VisibilityConstraint* rhs)
{
    if (n.has_child("target_radius"))
        n["target_radius"] >> rhs->target_radius;
    if (n.has_child("target_pose"))
        n["target_pose"] >> rhs->target_pose;
    if (n.has_child("cone_sides"))
        n["cone_sides"] >> rhs->cone_sides;
    if (n.has_child("sensor_pose"))
        n["sensor_pose"] >> rhs->sensor_pose;
    if (n.has_child("max_view_angle"))
        n["max_view_angle"] >> rhs->max_view_angle;
    if (n.has_child("max_range_angle"))
        n["max_range_angle"] >> rhs->max_range_angle;
    if (n.has_child("sensor_view_direction"))
        n["sensor_view_direction"] >> rhs->sensor_view_direction;
    if (n.has_child("weight"))
        n["weight"] >> rhs->weight;

    return true;
}

}  // namespace yml
}  // namespace c4
