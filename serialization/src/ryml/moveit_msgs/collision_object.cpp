#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/std_msgs/header.h>
#include <moveit_serialization/ryml/object_recognition_msgs/object_type.h>
#include <moveit_serialization/ryml/geometry_msgs/pose.h>
#include <moveit_serialization/ryml/shape_msgs/mesh.h>
#include <moveit_serialization/ryml/shape_msgs/solid_primitive.h>
#include <moveit_serialization/ryml/shape_msgs/plane.h>

#include <moveit_serialization/ryml/moveit_msgs/collision_object.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::CollisionObject const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("header") << rhs.header;
    n->append_child() << yml::key("pose") << rhs.pose;
    n->append_child() << yml::key("id") << rhs.id;
    n->append_child() << yml::key("type") << rhs.type;
    n->append_child() << yml::key("primitives") << rhs.primitives;
    n->append_child() << yml::key("primitive_poses") << rhs.primitive_poses;
    n->append_child() << yml::key("meshes") << rhs.meshes;
    n->append_child() << yml::key("mesh_poses") << rhs.mesh_poses;
    n->append_child() << yml::key("planes") << rhs.planes;
    n->append_child() << yml::key("plane_poses") << rhs.plane_poses;
    n->append_child() << yml::key("subframe_names") << rhs.subframe_names;
    n->append_child() << yml::key("subframe_poses") << rhs.subframe_poses;
    n->append_child() << yml::key("operation") << rhs.operation;
}

bool read(c4::yml::NodeRef const& n, moveit_msgs::CollisionObject* rhs)
{
    if (n.has_child("header"))
        n["header"] >> rhs->header;
    if (n.has_child("pose"))
        n["pose"] >> rhs->pose;
    if (n.has_child("id"))
        n["id"] >> rhs->id;
    if (n.has_child("type"))
        n["type"] >> rhs->type;
    if (n.has_child("primitives"))
        n["primitives"] >> rhs->primitives;
    if (n.has_child("primitive_poses"))
        n["primitive_poses"] >> rhs->primitive_poses;
    if (n.has_child("meshes"))
        n["meshes"] >> rhs->meshes;
    if (n.has_child("mesh_poses"))
        n["mesh_poses"] >> rhs->mesh_poses;
    if (n.has_child("planes"))
        n["planes"] >> rhs->planes;
    if (n.has_child("plane_poses"))
        n["plane_poses"] >> rhs->plane_poses;
    if (n.has_child("subframe_names"))
        n["subframe_names"] >> rhs->subframe_names;
    if (n.has_child("subframe_poses"))
        n["subframe_poses"] >> rhs->subframe_poses;
    if (n.has_child("operation"))
        n["operation"] >> rhs->operation;

    return true;
}

}  // namespace yml
}  // namespace c4
