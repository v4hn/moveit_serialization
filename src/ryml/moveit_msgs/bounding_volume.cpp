#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/geometry_msgs/pose.h>
#include <moveit_serialization/ryml/shape_msgs/solid_primitive.h>
#include <moveit_serialization/ryml/shape_msgs/mesh.h>

#include <moveit_serialization/ryml/moveit_msgs/bounding_volume.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_msgs::BoundingVolume const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("primitives") << rhs.primitives;
    n->append_child() << yml::key("primitive_poses") << rhs.primitive_poses;
    n->append_child() << yml::key("meshes") << rhs.meshes;
    n->append_child() << yml::key("mesh_poses") << rhs.mesh_poses;
}

bool read(c4::yml::ConstNodeRef const& n, moveit_msgs::BoundingVolume* rhs)
{
    if (n.has_child("primitives"))
        n["primitives"] >> rhs->primitives;
    if (n.has_child("primitive_poses"))
        n["primitive_poses"] >> rhs->primitive_poses;
    if (n.has_child("meshes"))
        n["meshes"] >> rhs->meshes;
    if (n.has_child("mesh_poses"))
        n["mesh_poses"] >> rhs->mesh_poses;

    return true;
}

}  // namespace yml
}  // namespace c4
