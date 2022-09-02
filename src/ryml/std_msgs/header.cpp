#include <moveit_serialization/ryml/std/std.h>

#include <moveit_serialization/ryml/std_msgs/header.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, std_msgs::Header const& rhs)
{
    *n |= c4::yml::MAP;

    n->append_child({ yml::KEYMAP, "stamp" });
    auto c = n->last_child();
    c.append_child() << yml::key("sec") << rhs.stamp.sec;
    c.append_child() << yml::key("nsec") << rhs.stamp.nsec;

    n->append_child() << yml::key("seq") << rhs.seq;
    n->append_child() << yml::key("frame_id") << rhs.frame_id;
}

bool read(c4::yml::ConstNodeRef const& n, std_msgs::Header* rhs)
{
    n["frame_id"] >> rhs->frame_id;

    if (n.has_child("seq"))
        n["seq"] >> rhs->seq;
    if (n.has_child("stamp")) {
        n["stamp"]["sec"] >> rhs->stamp.sec;
        n["stamp"]["nsec"] >> rhs->stamp.nsec;
    }

    return true;
}

}  // namespace yml
}  // namespace c4
