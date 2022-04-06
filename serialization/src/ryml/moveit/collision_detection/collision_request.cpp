#include <moveit_serialization/ryml/std.h>

#include <moveit_serialization/ryml/moveit/collision_detection/collision_request.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, collision_detection::CollisionRequest const& rhs)
{
    *n |= c4::yml::MAP;

    // Faster using the index API ~125% faster the the node API
    // See: https://github.com/biojppm/rapidyaml/issues/242
    // Tree* t = n->tree();
    // t->to_keyval(t->append_child(0), "distance", t->to_arena(rhs.distance));
    // t->to_keyval(t->append_child(0), "cost", t->to_arena(rhs.cost));
    // t->to_keyval(t->append_child(0), "contacts", t->to_arena(rhs.contacts));
    // t->to_keyval(t->append_child(0), "max_contacts", t->to_arena(rhs.max_contacts));
    // t->to_keyval(t->append_child(0), "max_contacts_per_pair", t->to_arena(rhs.max_contacts_per_pair));
    // t->to_keyval(t->append_child(0), "max_cost_sources", t->to_arena(rhs.max_cost_sources));
    // t->to_keyval(t->append_child(0), "verbose", t->to_arena(rhs.verbose));
    // t->to_keyval(t->append_child(0), "group_name", t->to_arena(rhs.group_name));

    n->append_child() << yml::key("distance") << rhs.distance;
    n->append_child() << yml::key("cost") << rhs.cost;
    n->append_child() << yml::key("contacts") << rhs.contacts;
    n->append_child() << yml::key("max_contacts") << rhs.max_contacts;
    n->append_child() << yml::key("max_contacts_per_pair") << rhs.max_contacts_per_pair;
    n->append_child() << yml::key("max_cost_sources") << rhs.max_cost_sources;
    n->append_child() << yml::key("verbose") << rhs.verbose;
    n->append_child() << yml::key("group_name") << rhs.group_name;
}

bool read(c4::yml::NodeRef const& n, collision_detection::CollisionRequest* rhs)
{
    n["distance"] >> rhs->distance;
    n["cost"] >> rhs->cost;
    n["contacts"] >> rhs->contacts;
    n["max_contacts"] >> rhs->max_contacts;
    n["max_contacts_per_pair"] >> rhs->max_contacts_per_pair;
    n["max_cost_sources"] >> rhs->max_cost_sources;
    n["verbose"] >> rhs->verbose;
    n["group_name"] >> rhs->group_name;

    return true;
}
}  // namespace yml
}  // namespace c4
