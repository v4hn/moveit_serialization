#include <moveit_serialization/ryml/format.h>
#include <moveit_serialization/ryml/std_msgs/color_rgba.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, std_msgs::ColorRGBA const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("r") << freal(rhs.r);
    n->append_child() << yml::key("g") << freal(rhs.g);
    n->append_child() << yml::key("b") << freal(rhs.b);
    n->append_child() << yml::key("a") << freal(rhs.a);
}

bool read(c4::yml::NodeRef const& n, std_msgs::ColorRGBA* rhs)
{
    n["r"] >> rhs->r;
    n["g"] >> rhs->g;
    n["b"] >> rhs->b;
    n["a"] >> rhs->a;

    return true;
}

}  // namespace yml
}  // namespace c4
