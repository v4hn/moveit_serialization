#include <moveit_serialization/ryml/format.h>
#include <moveit_serialization/ryml/shape_msgs/plane.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, shape_msgs::Plane const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child({ yml::KEYSEQ, "coef" }) |= yml::_WIP_STYLE_FLOW_SL;
    auto c = n->last_child();

    for (auto const& val : rhs.coef)
        c.append_child() << freal(val);
}

bool read(c4::yml::ConstNodeRef const& n, shape_msgs::Plane* rhs)
{
    n["coef"][0] >> rhs->coef[0];
    n["coef"][1] >> rhs->coef[1];
    n["coef"][2] >> rhs->coef[2];
    n["coef"][3] >> rhs->coef[3];

    return true;
}

}  // namespace yml
}  // namespace c4
