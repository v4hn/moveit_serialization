#include <moveit_serialization/yaml-cpp/conversion/object_recognition_msgs.h>

namespace YAML {

Node convert<object_recognition_msgs::ObjectType>::encode(const object_recognition_msgs::ObjectType& rhs)
{
    Node node;
    node["key"] = rhs.key;
    node["db"] = rhs.db;
    return node;
}

bool convert<object_recognition_msgs::ObjectType>::decode(const Node& node, object_recognition_msgs::ObjectType& rhs)
{
    rhs = object_recognition_msgs::ObjectType();

    if (node["key"])
        rhs.key = node["key"].as<std::string>();

    if (node["db"])
        rhs.db = node["db"].as<std::string>();

    return true;
}

}  // namespace YAML
