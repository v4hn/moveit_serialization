#include <moveit_serialization/yaml-cpp/conversion/moveit_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/std_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/sensor_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/trajectory_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/octomap_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/object_recognition_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/shape_msgs.h>
#include <moveit_serialization/yaml-cpp/conversion/geometry_msgs.h>

#include <moveit_serialization/yaml-cpp/conversion/utilities.h>

namespace YAML {

Node convert<moveit_msgs::PlanningScene>::encode(const moveit_msgs::PlanningScene& rhs) {
	Node node;
	node["name"] = rhs.name;
	node["robot_state"] = rhs.robot_state;
	node["robot_model_name"] = rhs.robot_model_name;
	node["fixed_frame_transforms"] = rhs.fixed_frame_transforms;
	node["allowed_collision_matrix"] = rhs.allowed_collision_matrix;

	// node["link_padding"] = rhs.link_padding;
	// node["link_padding"].SetStyle(YAML::EmitterStyle::Flow);

	// node["link_scale"] = rhs.link_scale;
	// node["link_scale"].SetStyle(YAML::EmitterStyle::Flow);

	// node["object_colors"] = rhs.object_colors;

	if (!rhs.world.collision_objects.empty() || !rhs.world.octomap.octomap.data.empty())
		node["world"] = rhs.world;

	if (rhs.is_diff)
		node["is_diff"] = boolToString(rhs.is_diff);

	return node;
}

bool convert<moveit_msgs::PlanningScene>::decode(const Node& node, moveit_msgs::PlanningScene& rhs) {
	rhs = moveit_msgs::PlanningScene();

	if (node["name"])
		rhs.name = node["name"].as<std::string>();

	if (node["robot_state"])
		rhs.robot_state = node["robot_state"].as<moveit_msgs::RobotState>();

	if (node["robot_model_name"])
		rhs.robot_model_name = node["robot_model_name"].as<std::string>();

	if (node["fixed_frame_transforms"])
		rhs.fixed_frame_transforms = node["fixed_frame_transforms"].as<std::vector<geometry_msgs::TransformStamped>>();

	if (node["allowed_collision_matrix"])
		rhs.allowed_collision_matrix = node["allowed_collision_matrix"].as<moveit_msgs::AllowedCollisionMatrix>();

	if (node["link_padding"])
		rhs.link_padding = node["link_padding"].as<std::vector<moveit_msgs::LinkPadding>>();

	if (node["link_scale"])
		rhs.link_scale = node["link_scale"].as<std::vector<moveit_msgs::LinkScale>>();

	if (node["object_colors"])
		rhs.object_colors = node["object_colors"].as<std::vector<moveit_msgs::ObjectColor>>();

	if (node["world"])
		rhs.world = node["world"].as<moveit_msgs::PlanningSceneWorld>();

	if (node["is_diff"])
		rhs.is_diff = nodeToBool(node["is_diff"]);

	return true;
}

Node convert<moveit_msgs::RobotState>::encode(const moveit_msgs::RobotState& rhs) {
	Node node;

	if (!rhs.joint_state.name.empty())
		node["joint_state"] = rhs.joint_state;

	if (!rhs.multi_dof_joint_state.joint_names.empty())
		node["multi_dof_joint_state"] = rhs.multi_dof_joint_state;

	if (!rhs.attached_collision_objects.empty())
		node["attached_collision_objects"] = rhs.attached_collision_objects;

	if (rhs.is_diff)
		node["is_diff"] = boolToString(rhs.is_diff);

	return node;
}

bool convert<moveit_msgs::RobotState>::decode(const Node& node, moveit_msgs::RobotState& rhs) {
	rhs = moveit_msgs::RobotState();

	if (node["joint_state"])
		rhs.joint_state = node["joint_state"].as<sensor_msgs::JointState>();

	if (node["multi_dof_joint_state"])
		rhs.multi_dof_joint_state = node["multi_dof_joint_state"].as<sensor_msgs::MultiDOFJointState>();

	if (node["attached_collision_objects"])
		rhs.attached_collision_objects =
		    node["attached_collision_objects"].as<std::vector<moveit_msgs::AttachedCollisionObject>>();

	if (node["is_diff"])
		rhs.is_diff = nodeToBool(node["is_diff"]);

	return true;
}

Node convert<moveit_msgs::AttachedCollisionObject>::encode(const moveit_msgs::AttachedCollisionObject& rhs) {
	Node node;
	node["link_name"] = rhs.link_name;
	node["object"] = rhs.object;

	if (!rhs.touch_links.empty())
		node["touch_links"] = rhs.touch_links;

	if (!rhs.detach_posture.points.empty())
		node["detach_posture"] = rhs.detach_posture;

	node["weight"] = rhs.weight;
	return node;
}

bool convert<moveit_msgs::AttachedCollisionObject>::decode(const Node& node,
                                                           moveit_msgs::AttachedCollisionObject& rhs) {
	rhs = moveit_msgs::AttachedCollisionObject();

	if (node["link_name"])
		rhs.link_name = node["link_name"].as<std::string>();

	if (node["object"])
		rhs.object = node["object"].as<moveit_msgs::CollisionObject>();

	if (node["touch_links"])
		rhs.touch_links = node["touch_links"].as<std::vector<std::string>>();

	if (node["detach_posture"])
		rhs.detach_posture = node["detach_posture"].as<trajectory_msgs::JointTrajectory>();

	if (node["weight"])
		rhs.weight = node["weight"].as<double>();

	return true;
}

Node convert<moveit_msgs::CollisionObject>::encode(const moveit_msgs::CollisionObject& rhs) {
	Node node;

	if (!isHeaderEmpty(rhs.header))
		node["header"] = rhs.header;

	node["id"] = rhs.id;

	if (!rhs.type.key.empty())
		node["type"] = rhs.type;

	node["pose"] = rhs.pose;

	if (!rhs.primitives.empty()) {
		node["primitives"] = rhs.primitives;
		node["primitive_poses"] = rhs.primitive_poses;
	}

	if (!rhs.meshes.empty()) {
		node["meshes"] = rhs.meshes;
		node["mesh_poses"] = rhs.mesh_poses;
	}

	if (!rhs.planes.empty()) {
		node["planes"] = rhs.planes;
		node["plane_poses"] = rhs.plane_poses;
	}

	std::string s;
	switch (rhs.operation) {
		case moveit_msgs::CollisionObject::REMOVE:
			s = "remove";
			break;
		case moveit_msgs::CollisionObject::APPEND:
			s = "append";
			break;
		case moveit_msgs::CollisionObject::MOVE:
			s = "move";
			break;
		default:
			return node;
	}

	node["operation"] = s;
	return node;
}

bool convert<moveit_msgs::CollisionObject>::decode(const Node& node, moveit_msgs::CollisionObject& rhs) {
	rhs = moveit_msgs::CollisionObject();

	if (node["header"])
		rhs.header = node["header"].as<std_msgs::Header>();
	else
		rhs.header = getDefaultHeader();

	if (node["id"])
		rhs.id = node["id"].as<std::string>();

	if (node["type"])
		rhs.type = node["type"].as<object_recognition_msgs::ObjectType>();

	if (node["pose"])
		rhs.pose = node["pose"].as<geometry_msgs::Pose>();

	if (node["primitives"]) {
		rhs.primitives = node["primitives"].as<std::vector<shape_msgs::SolidPrimitive>>();
		rhs.primitive_poses = node["primitive_poses"].as<std::vector<geometry_msgs::Pose>>();
	}

	if (node["meshes"]) {
		rhs.meshes = node["meshes"].as<std::vector<shape_msgs::Mesh>>();
		rhs.mesh_poses = node["mesh_poses"].as<std::vector<geometry_msgs::Pose>>();
	}

	if (node["planes"]) {
		rhs.planes = node["planes"].as<std::vector<shape_msgs::Plane>>();
		rhs.plane_poses = node["plane_poses"].as<std::vector<geometry_msgs::Pose>>();
	}

	if (node["operation"])
		rhs.operation = nodeToCollisionObject(node["operation"]);

	return true;
}

Node convert<moveit_msgs::LinkPadding>::encode(const moveit_msgs::LinkPadding& rhs) {
	Node node;
	node["link_name"] = rhs.link_name;
	node["padding"] = rhs.padding;
	return node;
}

bool convert<moveit_msgs::LinkPadding>::decode(const Node& node, moveit_msgs::LinkPadding& rhs) {
	rhs = moveit_msgs::LinkPadding();

	if (node["link_name"])
		rhs.link_name = node["link_name"].as<std::string>();

	if (node["padding"])
		rhs.padding = node["padding"].as<double>();

	return true;
}

Node convert<moveit_msgs::LinkScale>::encode(const moveit_msgs::LinkScale& rhs) {
	Node node;
	node["link_name"] = rhs.link_name;
	node["scale"] = rhs.scale;
	return node;
}

bool convert<moveit_msgs::LinkScale>::decode(const Node& node, moveit_msgs::LinkScale& rhs) {
	rhs = moveit_msgs::LinkScale();

	if (node["link_name"])
		rhs.link_name = node["link_name"].as<std::string>();

	if (node["scale"])
		rhs.scale = node["scale"].as<double>();

	return true;
}

Node convert<moveit_msgs::ObjectColor>::encode(const moveit_msgs::ObjectColor& rhs) {
	Node node;
	node["id"] = rhs.id;
	node["color"] = rhs.color;
	return node;
}

bool convert<moveit_msgs::ObjectColor>::decode(const Node& node, moveit_msgs::ObjectColor& rhs) {
	rhs = moveit_msgs::ObjectColor();

	if (node["id"])
		rhs.id = node["id"].as<std::string>();

	if (node["color"])
		rhs.color = node["color"].as<std_msgs::ColorRGBA>();

	return true;
}

Node convert<moveit_msgs::AllowedCollisionMatrix>::encode(const moveit_msgs::AllowedCollisionMatrix& rhs) {
	Node node;
	node["entry_names"] = rhs.entry_names;
	node["entry_names"].SetStyle(EmitterStyle::Flow);

	node["entry_values"] = rhs.entry_values;

	if (!rhs.default_entry_values.empty()) {
		node["default_entry_names"] = rhs.entry_names;
		node["default_entry_names"].SetStyle(EmitterStyle::Flow);

		std::vector<std::string> default_entry_values;
		for (const auto& b : rhs.default_entry_values)
			default_entry_values.emplace_back(boolToString(b));

		node["default_entry_values"] = default_entry_values;
		node["default_entry_values"].SetStyle(EmitterStyle::Flow);
	}

	return node;
}

bool convert<moveit_msgs::AllowedCollisionMatrix>::decode(const Node& node, moveit_msgs::AllowedCollisionMatrix& rhs) {
	rhs = moveit_msgs::AllowedCollisionMatrix();

	if (node["entry_names"])
		rhs.entry_names = node["entry_names"].as<std::vector<std::string>>();

	if (node["entry_values"])
		rhs.entry_values = node["entry_values"].as<std::vector<moveit_msgs::AllowedCollisionEntry>>();

	if (node["default_entry_names"])
		rhs.default_entry_names = node["default_entry_names"].as<std::vector<std::string>>();

	if (node["default_entry_values"]) {
		const auto& dev = node["default_entry_values"];
		for (const auto& b : dev)
			rhs.default_entry_values.push_back(nodeToBool(b));
	}

	return true;
}

Node convert<moveit_msgs::AllowedCollisionEntry>::encode(const moveit_msgs::AllowedCollisionEntry& rhs) {
	Node node;
	std::vector<std::string> enabled;
	for (const auto& b : rhs.enabled)
		enabled.emplace_back(boolToString(b));

	node = enabled;
	node.SetStyle(EmitterStyle::Flow);
	return node;
}

bool convert<moveit_msgs::AllowedCollisionEntry>::decode(const Node& node, moveit_msgs::AllowedCollisionEntry& rhs) {
	rhs = moveit_msgs::AllowedCollisionEntry();

	for (const auto& b : node)
		rhs.enabled.push_back(nodeToBool(b));

	return true;
}

Node convert<moveit_msgs::PlanningSceneWorld>::encode(const moveit_msgs::PlanningSceneWorld& rhs) {
	Node node;

	if (!rhs.collision_objects.empty())
		node["collision_objects"] = rhs.collision_objects;

	if (!rhs.octomap.octomap.data.empty())
		node["octomap"] = rhs.octomap;

	return node;
}

bool convert<moveit_msgs::PlanningSceneWorld>::decode(const Node& node, moveit_msgs::PlanningSceneWorld& rhs) {
	rhs = moveit_msgs::PlanningSceneWorld();

	if (node["collision_objects"])
		rhs.collision_objects = node["collision_objects"].as<std::vector<moveit_msgs::CollisionObject>>();

	if (node["octomap"])
		rhs.octomap = node["octomap"].as<octomap_msgs::OctomapWithPose>();

	return true;
}

Node convert<moveit_msgs::WorkspaceParameters>::encode(const moveit_msgs::WorkspaceParameters& rhs) {
	Node node;
	if (!isHeaderEmpty(rhs.header))
		node["header"] = rhs.header;

	node["min_corner"] = rhs.min_corner;
	node["max_corner"] = rhs.max_corner;
	return node;
}

bool convert<moveit_msgs::WorkspaceParameters>::decode(const Node& node, moveit_msgs::WorkspaceParameters& rhs) {
	rhs = moveit_msgs::WorkspaceParameters();

	if (node["header"])
		rhs.header = node["header"].as<std_msgs::Header>();
	else
		rhs.header = getDefaultHeader();

	rhs.min_corner = node["min_corner"].as<geometry_msgs::Vector3>();
	rhs.max_corner = node["max_corner"].as<geometry_msgs::Vector3>();
	return true;
}

Node convert<moveit_msgs::Constraints>::encode(const moveit_msgs::Constraints& rhs) {
	Node node;

	if (!rhs.name.empty())
		node["name"] = rhs.name;

	if (!rhs.joint_constraints.empty())
		node["joint_constraints"] = rhs.joint_constraints;

	if (!rhs.position_constraints.empty())
		node["position_constraints"] = rhs.position_constraints;

	if (!rhs.orientation_constraints.empty())
		node["orientation_constraints"] = rhs.orientation_constraints;

	if (!rhs.visibility_constraints.empty())
		node["visibility_constraints"] = rhs.visibility_constraints;

	return node;
}

bool convert<moveit_msgs::Constraints>::decode(const Node& node, moveit_msgs::Constraints& rhs) {
	rhs = moveit_msgs::Constraints();

	if (node["name"])
		rhs.name = node["name"].as<std::string>();

	if (node["joint_constraints"])
		rhs.joint_constraints = node["joint_constraints"].as<std::vector<moveit_msgs::JointConstraint>>();

	if (node["position_constraints"])
		rhs.position_constraints = node["position_constraints"].as<std::vector<moveit_msgs::PositionConstraint>>();

	if (node["orientation_constraints"])
		rhs.orientation_constraints =
		    node["orientation_constraints"].as<std::vector<moveit_msgs::OrientationConstraint>>();

	if (node["visibility_constraints"])
		rhs.visibility_constraints = node["visibility_constraints"].as<std::vector<moveit_msgs::VisibilityConstraint>>();

	return true;
}

Node convert<moveit_msgs::JointConstraint>::encode(const moveit_msgs::JointConstraint& rhs) {
	Node node;
	node["joint_name"] = rhs.joint_name;
	node["position"] = rhs.position;

	if (rhs.tolerance_above > std::numeric_limits<double>::epsilon())
		node["tolerance_above"] = rhs.tolerance_above;

	if (rhs.tolerance_below > std::numeric_limits<double>::epsilon())
		node["tolerance_below"] = rhs.tolerance_below;

	// if (rhs.weight < 1)
	node["weight"] = rhs.weight;

	return node;
}

bool convert<moveit_msgs::JointConstraint>::decode(const Node& node, moveit_msgs::JointConstraint& rhs) {
	rhs.joint_name = node["joint_name"].as<std::string>();
	rhs.position = node["position"].as<double>();

	if (node["tolerance_above"])
		rhs.tolerance_above = node["tolerance_above"].as<double>();
	else
		rhs.tolerance_above = std::numeric_limits<double>::epsilon();

	if (node["tolerance_below"])
		rhs.tolerance_below = node["tolerance_below"].as<double>();
	else
		rhs.tolerance_below = std::numeric_limits<double>::epsilon();

	if (node["weight"])
		rhs.weight = node["weight"].as<double>();
	else
		rhs.weight = 1;

	return true;
}

Node convert<moveit_msgs::PositionConstraint>::encode(const moveit_msgs::PositionConstraint& rhs) {
	Node node;
	if (!isHeaderEmpty(rhs.header))
		node["header"] = rhs.header;

	node["link_name"] = rhs.link_name;

	// if (!isVector3Zero(rhs.target_point_offset))
	node["target_point_offset"] = rhs.target_point_offset;

	node["constraint_region"] = rhs.constraint_region;

	// if (rhs.weight < 1)
	node["weight"] = rhs.weight;

	return node;
}

bool convert<moveit_msgs::PositionConstraint>::decode(const Node& node, moveit_msgs::PositionConstraint& rhs) {
	rhs = moveit_msgs::PositionConstraint();
	rhs.weight = 1;

	if (node["header"])
		rhs.header = node["header"].as<std_msgs::Header>();
	else
		rhs.header = getDefaultHeader();

	rhs.link_name = node["link_name"].as<std::string>();
	if (node["target_point_offset"])
		rhs.target_point_offset = node["target_point_offset"].as<geometry_msgs::Vector3>();

	rhs.constraint_region = node["constraint_region"].as<moveit_msgs::BoundingVolume>();
	if (node["weight"])
		rhs.weight = node["weight"].as<double>();
	else
		rhs.weight = 1;

	return true;
}

Node convert<moveit_msgs::OrientationConstraint>::encode(const moveit_msgs::OrientationConstraint& rhs) {
	Node node;

	if (!isHeaderEmpty(rhs.header))
		node["header"] = rhs.header;

	node["orientation"] = rhs.orientation;
	node["link_name"] = rhs.link_name;

	node["absolute_x_axis_tolerance"] = rhs.absolute_x_axis_tolerance;
	node["absolute_y_axis_tolerance"] = rhs.absolute_y_axis_tolerance;
	node["absolute_z_axis_tolerance"] = rhs.absolute_z_axis_tolerance;

	node["parameterization"] = rhs.parameterization;

	// if (rhs.weight < 1)
	node["weight"] = rhs.weight;

	return node;
}

bool convert<moveit_msgs::OrientationConstraint>::decode(const Node& node, moveit_msgs::OrientationConstraint& rhs) {
	if (node["header"])
		rhs.header = node["header"].as<std_msgs::Header>();
	else
		rhs.header = getDefaultHeader();

	rhs.orientation = node["orientation"].as<geometry_msgs::Quaternion>();
	rhs.link_name = node["link_name"].as<std::string>();

	rhs.absolute_x_axis_tolerance = node["absolute_x_axis_tolerance"].as<double>();
	rhs.absolute_y_axis_tolerance = node["absolute_y_axis_tolerance"].as<double>();
	rhs.absolute_z_axis_tolerance = node["absolute_z_axis_tolerance"].as<double>();

	if (node["parameterization"])
		rhs.parameterization = node["parameterization"].as<uint8_t>();
	else
		rhs.parameterization = rhs.XYZ_EULER_ANGLES;

	if (node["weight"])
		rhs.weight = node["weight"].as<double>();
	else
		rhs.weight = 1;

	return true;
}

Node convert<moveit_msgs::VisibilityConstraint>::encode(const moveit_msgs::VisibilityConstraint& rhs) {
	Node node;
	node["target_radius"] = rhs.target_radius;
	node["target_pose"] = rhs.target_pose;
	node["cone_sides"] = rhs.cone_sides;
	node["sensor_pose"] = rhs.sensor_pose;
	node["max_view_angle"] = rhs.max_view_angle;
	node["max_range_angle"] = rhs.max_range_angle;
	node["sensor_view_direction"] = rhs.sensor_view_direction;
	node["weight"] = rhs.weight;
	return node;
}

bool convert<moveit_msgs::VisibilityConstraint>::decode(const Node& node, moveit_msgs::VisibilityConstraint& rhs) {
	rhs = moveit_msgs::VisibilityConstraint();

	rhs.target_radius = node["target_radius"].as<double>();
	rhs.target_pose = node["target_pose"].as<geometry_msgs::PoseStamped>();
	rhs.cone_sides = node["cone_sides"].as<int>();
	rhs.sensor_pose = node["sensor_pose"].as<geometry_msgs::PoseStamped>();
	rhs.max_view_angle = node["max_view_angle"].as<double>();
	rhs.max_range_angle = node["max_range_angle"].as<double>();
	rhs.sensor_view_direction = node["sensor_view_direction"].as<int>();
	rhs.weight = node["weight"].as<double>();

	return true;
}

Node convert<moveit_msgs::BoundingVolume>::encode(const moveit_msgs::BoundingVolume& rhs) {
	Node node;

	if (!rhs.primitives.empty()) {
		node["primitives"] = rhs.primitives;
		node["primitive_poses"] = rhs.primitive_poses;
	}

	if (!rhs.meshes.empty()) {
		node["meshes"] = rhs.meshes;
		node["mesh_poses"] = rhs.mesh_poses;
	}

	return node;
}

bool convert<moveit_msgs::BoundingVolume>::decode(const Node& node, moveit_msgs::BoundingVolume& rhs) {
	rhs = moveit_msgs::BoundingVolume();

	if (node["primitives"]) {
		rhs.primitives = node["primitives"].as<std::vector<shape_msgs::SolidPrimitive>>();
		rhs.primitive_poses = node["primitive_poses"].as<std::vector<geometry_msgs::Pose>>();
	}

	if (node["meshes"]) {
		rhs.meshes = node["meshes"].as<std::vector<shape_msgs::Mesh>>();
		rhs.mesh_poses = node["mesh_poses"].as<std::vector<geometry_msgs::Pose>>();
	}

	return true;
}

Node convert<moveit_msgs::TrajectoryConstraints>::encode(const moveit_msgs::TrajectoryConstraints& rhs) {
	Node node;
	node["constraints"] = rhs.constraints;
	return node;
}

bool convert<moveit_msgs::TrajectoryConstraints>::decode(const Node& node, moveit_msgs::TrajectoryConstraints& rhs) {
	rhs.constraints = node["constraints"].as<std::vector<moveit_msgs::Constraints>>();
	return true;
}

Node convert<moveit_msgs::MotionPlanRequest>::encode(const moveit_msgs::MotionPlanRequest& rhs) {
	Node node;

	if (!(isHeaderEmpty(rhs.workspace_parameters.header) && isVector3Zero(rhs.workspace_parameters.min_corner) &&
	      isVector3Zero(rhs.workspace_parameters.max_corner)))
		node["workspace_parameters"] = rhs.workspace_parameters;

	node["start_state"] = rhs.start_state;

	if (!rhs.goal_constraints.empty())
		node["goal_constraints"] = rhs.goal_constraints;

	if (!isConstraintEmpty(rhs.path_constraints))
		node["path_constraints"] = rhs.path_constraints;

	if (!rhs.trajectory_constraints.constraints.empty())
		node["trajectory_constraints"] = rhs.trajectory_constraints;

	if (!rhs.planner_id.empty())
		node["planner_id"] = rhs.planner_id;

	if (!rhs.group_name.empty())
		node["group_name"] = rhs.group_name;
	if (rhs.num_planning_attempts != 0)
		node["num_planning_attempts"] = rhs.num_planning_attempts;

	if (rhs.allowed_planning_time != 0)
		node["allowed_planning_time"] = rhs.allowed_planning_time;

	if (rhs.max_velocity_scaling_factor < 1)
		node["max_velocity_scaling_factor"] = rhs.max_velocity_scaling_factor;

	if (rhs.max_acceleration_scaling_factor < 1)
		node["max_acceleration_scaling_factor"] = rhs.max_acceleration_scaling_factor;

	return node;
}

bool convert<moveit_msgs::MotionPlanRequest>::decode(const Node& node, moveit_msgs::MotionPlanRequest& rhs) {
	rhs = moveit_msgs::MotionPlanRequest();

	if (node["workspace_parameters"])
		rhs.workspace_parameters = node["workspace_parameters"].as<moveit_msgs::WorkspaceParameters>();

	if (node["start_state"])
		rhs.start_state = node["start_state"].as<moveit_msgs::RobotState>();

	if (node["goal_constraints"])
		rhs.goal_constraints = node["goal_constraints"].as<std::vector<moveit_msgs::Constraints>>();

	if (node["path_constraints"])
		rhs.path_constraints = node["path_constraints"].as<moveit_msgs::Constraints>();

	if (node["trajectory_constraints"])
		rhs.trajectory_constraints = node["trajectory_constraints"].as<moveit_msgs::TrajectoryConstraints>();

	if (node["planner_id"])
		rhs.planner_id = node["planner_id"].as<std::string>();

	if (node["group_name"])
		rhs.group_name = node["group_name"].as<std::string>();

	if (node["num_planning_attempts"])
		rhs.num_planning_attempts = node["num_planning_attempts"].as<int>();
	else
		rhs.num_planning_attempts = 0;

	if (node["allowed_planning_time"])
		rhs.allowed_planning_time = node["allowed_planning_time"].as<double>();
	else
		rhs.allowed_planning_time = 0;

	if (node["max_velocity_scaling_factor"])
		rhs.max_velocity_scaling_factor = node["max_velocity_scaling_factor"].as<double>();
	else
		rhs.max_velocity_scaling_factor = 1;

	if (node["max_acceleration_scaling_factor"])
		rhs.max_acceleration_scaling_factor = node["max_acceleration_scaling_factor"].as<double>();
	else
		rhs.max_acceleration_scaling_factor = 1;

	return true;
}

Node convert<moveit_msgs::RobotTrajectory>::encode(const moveit_msgs::RobotTrajectory& rhs) {
	Node node;

	if (!rhs.joint_trajectory.points.empty())
		node["joint_trajectory"] = rhs.joint_trajectory;

	if (!rhs.multi_dof_joint_trajectory.points.empty())
		node["multi_dof_joint_trajectory"] = rhs.multi_dof_joint_trajectory;

	return node;
}

bool convert<moveit_msgs::RobotTrajectory>::decode(const Node& node, moveit_msgs::RobotTrajectory& rhs) {
	rhs = moveit_msgs::RobotTrajectory();

	if (node["joint_trajectory"])
		rhs.joint_trajectory = node["joint_trajectory"].as<trajectory_msgs::JointTrajectory>();

	if (node["multi_dof_joint_trajectory"])
		rhs.multi_dof_joint_trajectory =
		    node["multi_dof_joint_trajectory"].as<trajectory_msgs::MultiDOFJointTrajectory>();

	return true;
}

}  // namespace YAML
