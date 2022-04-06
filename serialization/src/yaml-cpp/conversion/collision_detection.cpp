#include <moveit_serialization/yaml-cpp/conversion/collision_detection.h>

namespace YAML {

Node convert<collision_detection::CollisionRequest>::encode(const collision_detection::CollisionRequest& rhs)
{
    Node node;

    node["distance"] = rhs.distance;
    node["cost"] = rhs.cost;
    node["contacts"] = rhs.contacts;
    node["max_contacts"] = rhs.max_contacts;
    node["max_contacts_per_pair"] = rhs.max_contacts_per_pair;
    node["max_cost_sources"] = rhs.max_cost_sources;
    node["verbose"] = rhs.verbose;
    node["group_name"] = rhs.group_name;

    return node;
}

bool convert<collision_detection::CollisionRequest>::decode(const Node& node, collision_detection::CollisionRequest& rhs)
{
    rhs.distance = node["distance"].as<bool>();
    rhs.cost = node["cost"].as<bool>();
    rhs.contacts = node["contacts"].as<bool>();
    rhs.max_contacts = node["max_contacts"].as<double>();
    rhs.max_contacts_per_pair = node["max_contacts_per_pair"].as<double>();
    rhs.max_cost_sources = node["max_cost_sources"].as<double>();
    rhs.verbose = node["verbose"].as<bool>();
    rhs.group_name = node["group_name"].as<std::string>("");

    return true;
}

}  // namespace YAML
