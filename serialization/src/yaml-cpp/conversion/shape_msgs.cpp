#include <moveit_serialization/yaml-cpp/conversion/shape_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/geometry_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/utilities.h>

#include <Eigen/Geometry>

namespace YAML {

Node convert<shape_msgs::SolidPrimitive>::encode(const shape_msgs::SolidPrimitive& rhs)
{
    Node node;
    encodeToUINT8(node, "type", rhs.type);

    node["dimensions"] = rhs.dimensions;
    node.SetStyle(EmitterStyle::Flow);
    return node;
}

bool convert<shape_msgs::SolidPrimitive>::decode(const Node& node, shape_msgs::SolidPrimitive& rhs)
{
    rhs = shape_msgs::SolidPrimitive();
    if (node["type"])
        decodeToUINT8(node, "type", rhs.type);

    if (node["dimensions"])
        rhs.dimensions = node["dimensions"].as<std::vector<double>>();

    return true;
}

Node convert<shape_msgs::Mesh>::encode(const shape_msgs::Mesh& rhs)
{
    Node node;
    node["triangles"] = rhs.triangles;
    node["vertices"] = rhs.vertices;

    return node;
}

bool convert<shape_msgs::Mesh>::decode(const Node& node, shape_msgs::Mesh& rhs)
{
    rhs = shape_msgs::Mesh();
    // if (node["resource"]) {
    // 	std::string resource = node["resource"].as<std::string>();
    // 	Eigen::Vector3d dimensions{ 1, 1, 1 };

    // 	if (node["dimensions"]) {
    // 		Eigen::Vector3d load(node["dimensions"].as<std::vector<double>>().data());
    // 		dimensions = load;
    // 	}

    // 	Geometry mesh(Geometry::ShapeType::Type::MESH, dimensions, resource);
    // 	rhs = mesh.getMeshMsg();
    // } else {
    if (node["triangles"])
        rhs.triangles = node["triangles"].as<std::vector<shape_msgs::MeshTriangle>>();
    if (node["vertices"])
        rhs.vertices = node["vertices"].as<std::vector<geometry_msgs::Point>>();
    // }
    return true;
}

Node convert<shape_msgs::MeshTriangle>::encode(const shape_msgs::MeshTriangle& rhs)
{
    Node node;
    node.push_back(rhs.vertex_indices[0]);
    node.push_back(rhs.vertex_indices[1]);
    node.push_back(rhs.vertex_indices[2]);
    node.SetStyle(EmitterStyle::Flow);

    return node;
}

bool convert<shape_msgs::MeshTriangle>::decode(const Node& node, shape_msgs::MeshTriangle& rhs)
{
    rhs.vertex_indices[0] = node[0].as<double>();
    rhs.vertex_indices[1] = node[1].as<double>();
    rhs.vertex_indices[2] = node[2].as<double>();
    return true;
}

Node convert<shape_msgs::Plane>::encode(const shape_msgs::Plane& rhs)
{
    Node node;
    node["coef"].push_back(rhs.coef[0]);
    node["coef"].push_back(rhs.coef[1]);
    node["coef"].push_back(rhs.coef[2]);
    node["coef"].push_back(rhs.coef[3]);
    node["coef"].SetStyle(EmitterStyle::Flow);
    return node;
}

bool convert<shape_msgs::Plane>::decode(const Node& node, shape_msgs::Plane& rhs)
{
    rhs.coef[0] = node["coef"][0].as<double>();
    rhs.coef[1] = node["coef"][1].as<double>();
    rhs.coef[2] = node["coef"][2].as<double>();
    rhs.coef[3] = node["coef"][3].as<double>();
    return true;
}

}  // namespace YAML
