#include <moveit_serialization/ryml/shape_msgs/mesh_triangle.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, shape_msgs::MeshTriangle const& rhs)
{
    *n |= c4::yml::SEQ;
    *n |= c4::yml::_WIP_STYLE_FLOW_SL;

    for (auto const& v : rhs.vertex_indices)
        n->append_child() << v;
}

bool read(c4::yml::ConstNodeRef const& n, shape_msgs::MeshTriangle* rhs)
{
    n[0] >> rhs->vertex_indices[0];
    n[1] >> rhs->vertex_indices[1];
    n[2] >> rhs->vertex_indices[2];

    return true;
}

}  // namespace yml
}  // namespace c4
