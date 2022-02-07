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
   Desc: geometry_msgs YAML conversion
*/

#pragma once

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <yaml-cpp/yaml.h>  // moveit_serialisation_yamlcpp

namespace YAML {

template <>
struct convert<geometry_msgs::TransformStamped>
{
	static Node encode(const geometry_msgs::TransformStamped& rhs);
	static bool decode(const Node& node, geometry_msgs::TransformStamped& rhs);
};

template <>
struct convert<geometry_msgs::Pose>
{
	static Node encode(const geometry_msgs::Pose& rhs);
	static bool decode(const Node& node, geometry_msgs::Pose& rhs);
};

template <>
struct convert<geometry_msgs::PoseStamped>
{
	static Node encode(const geometry_msgs::PoseStamped& rhs);
	static bool decode(const Node& node, geometry_msgs::PoseStamped& rhs);
};

template <>
struct convert<geometry_msgs::Transform>
{
	static Node encode(const geometry_msgs::Transform& rhs);
	static bool decode(const Node& node, geometry_msgs::Transform& rhs);
};

template <>
struct convert<geometry_msgs::Vector3>
{
	static Node encode(const geometry_msgs::Vector3& rhs);
	static bool decode(const Node& node, geometry_msgs::Vector3& rhs);
};

template <>
struct convert<geometry_msgs::Point>
{
	static Node encode(const geometry_msgs::Point& rhs);
	static bool decode(const Node& node, geometry_msgs::Point& rhs);
};

template <>
struct convert<geometry_msgs::Quaternion>
{
	static Node encode(const geometry_msgs::Quaternion& rhs);
	static bool decode(const Node& node, geometry_msgs::Quaternion& rhs);
};

template <>
struct convert<geometry_msgs::Twist>
{
	static Node encode(const geometry_msgs::Twist& rhs);
	static bool decode(const Node& node, geometry_msgs::Twist& rhs);
};

template <>
struct convert<geometry_msgs::Wrench>
{
	static Node encode(const geometry_msgs::Wrench& rhs);
	static bool decode(const Node& node, geometry_msgs::Wrench& rhs);
};

}  // namespace YAML
