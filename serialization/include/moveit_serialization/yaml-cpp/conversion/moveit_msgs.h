/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Captain Yoshi
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Captain Yoshi
   Desc: moveit_msgs YAML conversion
*/

#pragma once

#include <moveit_msgs/LinkPadding.h>
#include <moveit_msgs/LinkScale.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit_msgs/BoundingVolume.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/VisibilityConstraint.h>
#include <moveit_msgs/TrajectoryConstraints.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/AllowedCollisionEntry.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <moveit_msgs/MotionPlanRequest.h>

#include <yaml-cpp/yaml.h>  // moveit_serialisation_yamlcpp

namespace YAML {

template <>
struct convert<moveit_msgs::PlanningScene>
{
    static Node encode(const moveit_msgs::PlanningScene& rhs);
    static bool decode(const Node& node, moveit_msgs::PlanningScene& rhs);
};

template <>
struct convert<moveit_msgs::RobotState>
{
    static Node encode(const moveit_msgs::RobotState& rhs);
    static bool decode(const Node& node, moveit_msgs::RobotState& rhs);
};

template <>
struct convert<moveit_msgs::AttachedCollisionObject>
{
    static Node encode(const moveit_msgs::AttachedCollisionObject& rhs);
    static bool decode(const Node& node, moveit_msgs::AttachedCollisionObject& rhs);
};

template <>
struct convert<moveit_msgs::CollisionObject>
{
    static Node encode(const moveit_msgs::CollisionObject& rhs);
    static bool decode(const Node& node, moveit_msgs::CollisionObject& rhs);
};

template <>
struct convert<moveit_msgs::LinkPadding>
{
    static Node encode(const moveit_msgs::LinkPadding& rhs);
    static bool decode(const Node& node, moveit_msgs::LinkPadding& rhs);
};

template <>
struct convert<moveit_msgs::LinkScale>
{
    static Node encode(const moveit_msgs::LinkScale& rhs);
    static bool decode(const Node& node, moveit_msgs::LinkScale& rhs);
};

template <>
struct convert<moveit_msgs::AllowedCollisionMatrix>
{
    static Node encode(const moveit_msgs::AllowedCollisionMatrix& rhs);
    static bool decode(const Node& node, moveit_msgs::AllowedCollisionMatrix& rhs);
};

template <>
struct convert<moveit_msgs::AllowedCollisionEntry>
{
    static Node encode(const moveit_msgs::AllowedCollisionEntry& rhs);
    static bool decode(const Node& node, moveit_msgs::AllowedCollisionEntry& rhs);
};

template <>
struct convert<moveit_msgs::PlanningSceneWorld>
{
    static Node encode(const moveit_msgs::PlanningSceneWorld& rhs);
    static bool decode(const Node& node, moveit_msgs::PlanningSceneWorld& rhs);
};

template <>
struct convert<moveit_msgs::ObjectColor>
{
    static Node encode(const moveit_msgs::ObjectColor& rhs);
    static bool decode(const Node& node, moveit_msgs::ObjectColor& rhs);
};

template <>
struct convert<moveit_msgs::WorkspaceParameters>
{
    static Node encode(const moveit_msgs::WorkspaceParameters& rhs);
    static bool decode(const Node& node, moveit_msgs::WorkspaceParameters& rhs);
};

template <>
struct convert<moveit_msgs::Constraints>
{
    static Node encode(const moveit_msgs::Constraints& rhs);
    static bool decode(const Node& node, moveit_msgs::Constraints& rhs);
};

template <>
struct convert<moveit_msgs::JointConstraint>
{
    static Node encode(const moveit_msgs::JointConstraint& rhs);
    static bool decode(const Node& node, moveit_msgs::JointConstraint& rhs);
};

template <>
struct convert<moveit_msgs::PositionConstraint>
{
    static Node encode(const moveit_msgs::PositionConstraint& rhs);
    static bool decode(const Node& node, moveit_msgs::PositionConstraint& rhs);
};

template <>
struct convert<moveit_msgs::OrientationConstraint>
{
    static Node encode(const moveit_msgs::OrientationConstraint& rhs);
    static bool decode(const Node& node, moveit_msgs::OrientationConstraint& rhs);
};

template <>
struct convert<moveit_msgs::VisibilityConstraint>
{
    static Node encode(const moveit_msgs::VisibilityConstraint& rhs);
    static bool decode(const Node& node, moveit_msgs::VisibilityConstraint& rhs);
};

template <>
struct convert<moveit_msgs::TrajectoryConstraints>
{
    static Node encode(const moveit_msgs::TrajectoryConstraints& rhs);
    static bool decode(const Node& node, moveit_msgs::TrajectoryConstraints& rhs);
};

template <>
struct convert<moveit_msgs::BoundingVolume>
{
    static Node encode(const moveit_msgs::BoundingVolume& rhs);
    static bool decode(const Node& node, moveit_msgs::BoundingVolume& rhs);
};

template <>
struct convert<moveit_msgs::MotionPlanRequest>
{
    static Node encode(const moveit_msgs::MotionPlanRequest& rhs);
    static bool decode(const Node& node, moveit_msgs::MotionPlanRequest& rhs);
};

template <>
struct convert<moveit_msgs::RobotTrajectory>
{
    static Node encode(const moveit_msgs::RobotTrajectory& rhs);
    static bool decode(const Node& node, moveit_msgs::RobotTrajectory& rhs);
};

}  // namespace YAML
