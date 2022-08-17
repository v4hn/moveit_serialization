#include <moveit_serialization/ryml/format.h>
#include <moveit_serialization/ryml/ros/time.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, ros::Time const& rhs)
{
    *n << freal(rhs.toSec());
}

bool read(c4::yml::ConstNodeRef const& n, ros::Time* rhs)
{
    double secs{ 0.0 };
    n >> secs;

    rhs->fromSec(secs);

    return true;
}

}  // namespace yml
}  // namespace c4
