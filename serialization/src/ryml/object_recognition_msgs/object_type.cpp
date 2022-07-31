#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/object_recognition_msgs/object_type.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, object_recognition_msgs::ObjectType const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child() << yml::key("key") << rhs.key;
    n->append_child() << yml::key("db") << rhs.db;
}

bool read(c4::yml::NodeRef const& n, object_recognition_msgs::ObjectType* rhs)
{
    if (n.has_child("key"))
        n["key"] >> rhs->key;
    if (n.has_child("vertices"))
        n["vertices"] >> rhs->db;

    return true;
}

}  // namespace yml
}  // namespace c4
