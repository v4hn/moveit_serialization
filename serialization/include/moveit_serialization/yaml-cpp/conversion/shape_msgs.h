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
   Desc: shape_msgs YAML conversion
*/

#pragma once

#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/MeshTriangle.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/Plane.h>

#include <yaml-cpp/yaml.h>  // moveit_serialisation_yamlcpp

namespace YAML {

template <>
struct convert<shape_msgs::SolidPrimitive>
{
    static Node encode(const shape_msgs::SolidPrimitive& rhs);
    static bool decode(const Node& node, shape_msgs::SolidPrimitive& rhs);
};

template <>
struct convert<shape_msgs::Mesh>
{
    static Node encode(const shape_msgs::Mesh& rhs);
    static bool decode(const Node& node, shape_msgs::Mesh& rhs);
};

template <>
struct convert<shape_msgs::MeshTriangle>
{
    static Node encode(const shape_msgs::MeshTriangle& rhs);
    static bool decode(const Node& node, shape_msgs::MeshTriangle& rhs);
};

template <>
struct convert<shape_msgs::Plane>
{
    static Node encode(const shape_msgs::Plane& rhs);
    static bool decode(const Node& node, shape_msgs::Plane& rhs);
};

}  // namespace YAML
