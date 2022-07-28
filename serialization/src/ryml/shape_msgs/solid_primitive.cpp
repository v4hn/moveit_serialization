#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/shape_msgs/solid_primitive.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, shape_msgs::SolidPrimitive const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("type") << rhs.type;
    n->append_child() << yml::key("dimensions") << rhs.dimensions |= yml::_WIP_STYLE_FLOW_SL;
}

bool read(c4::yml::NodeRef const& n, shape_msgs::SolidPrimitive* rhs)
{
    n["type"] >> rhs->type;
    n["dimensions"] >> rhs->dimensions;

    return true;
}

}  // namespace yml
}  // namespace c4
