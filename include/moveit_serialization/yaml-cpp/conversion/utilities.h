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
   Desc: Helper for YAML conversion
*/

#pragma once

#include <moveit_serialization/yaml-cpp/conversion/conversion.h>

namespace YAML {

/// Encode object to node
template <typename T>
Node toNode(const T& t)
{
    YAML::Node node;
    node = t;
    return node;
}

// WARNING
// Do not convert directly to/from uint8_t directly because
// yaml-cpp library converts them to un/signed char.
//
// See https://github.com/jbeder/yaml-cpp/issues/1081
// See https://www.boost.org/doc/libs/1_40_0/libs/conversion/lexical_cast.htm#faq
void encodeToUINT8(YAML::Node& n, const std::string& key, const uint8_t& data);
void decodeToUINT8(const YAML::Node& n, const std::string& key, uint8_t& data);

std::string boolToString(bool b);
bool nodeToBool(const YAML::Node& n);

bool isHeaderEmpty(const std_msgs::Header& h);
std_msgs::Header getDefaultHeader();

unsigned int nodeToCollisionObject(const YAML::Node& n);

bool isVector3Zero(const geometry_msgs::Vector3& v);
bool isConstraintEmpty(const moveit_msgs::Constraints& c);

std::string compressHex(const std::vector<int8_t>& v);
std::vector<int8_t> decompressHex(const std::string& hex);

}  // namespace YAML
