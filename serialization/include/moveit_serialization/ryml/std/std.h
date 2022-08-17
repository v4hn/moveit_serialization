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
   Desc: Default std header to force formating on floats
*/

#pragma once

#include <c4/yml/node.hpp>

#include <c4/yml/std/std.hpp>
#include <moveit_serialization/ryml/format.h>

#include <utility>

namespace c4 {
namespace yml {

// std::pair
template <class K, class V>
void write(c4::yml::NodeRef* n, std::pair<K, V> const& pair)
{
    *n |= c4::yml::SEQ;
    n->append_child() << pair.first;
    n->append_child() << pair.second;
}

template <class K, class V>
bool read(c4::yml::ConstNodeRef const& n, std::pair<K, V>* pair)
{
    if (!n.is_seq() || n.num_children() != 2)
        return false;

    n[0] >> pair->first;
    n[1] >> pair->second;

    return true;
}

// specialize vector of floats
template <class Alloc>
void write(c4::yml::NodeRef* n, std::vector<float, Alloc> const& vec)
{
    *n |= c4::yml::SEQ;
    for (auto const& v : vec) {
        n->append_child() << freal(v);
    }
}

template <class Alloc>
void write(c4::yml::NodeRef* n, std::vector<double, Alloc> const& vec)
{
    *n |= c4::yml::SEQ;
    for (auto const& v : vec) {
        n->append_child() << freal(v);
    }
}

// specialize map of floats (3^3 - 1 possibilities)
template <class V, class Less, class Alloc>
void write(c4::yml::NodeRef* n, std::map<float, V, Less, Alloc> const& m)
{
    *n |= c4::yml::MAP;
    for (auto const& p : m) {
        auto ch = n->append_child();
        auto r = freal(p.first);
        ch << c4::yml::key(r);
        ch << p.second;
    }
}

template <class Less, class Alloc>
void write(c4::yml::NodeRef* n, std::map<float, float, Less, Alloc> const& m)
{
    *n |= c4::yml::MAP;
    for (auto const& p : m) {
        auto ch = n->append_child();
        auto r = freal(p.first);
        ch << c4::yml::key(r);
        ch << freal(p.second);
    }
}

template <class Less, class Alloc>
void write(c4::yml::NodeRef* n, std::map<float, double, Less, Alloc> const& m)
{
    *n |= c4::yml::MAP;
    for (auto const& p : m) {
        auto ch = n->append_child();
        auto r = freal(p.first);
        ch << c4::yml::key(r);
        ch << freal(p.second);
    }
}

template <class K, class Less, class Alloc>
void write(c4::yml::NodeRef* n, std::map<K, float, Less, Alloc> const& m)
{
    *n |= c4::yml::MAP;
    for (auto const& p : m) {
        auto ch = n->append_child();
        ch << c4::yml::key(p.first);
        ch << freal(p.second);
    }
}

template <class V, class Less, class Alloc>
void write(c4::yml::NodeRef* n, std::map<double, V, Less, Alloc> const& m)
{
    *n |= c4::yml::MAP;
    for (auto const& p : m) {
        auto ch = n->append_child();
        auto r = freal(p.first);
        ch << c4::yml::key(r);
        ch << p.second;
    }
}

template <class Less, class Alloc>
void write(c4::yml::NodeRef* n, std::map<double, double, Less, Alloc> const& m)
{
    *n |= c4::yml::MAP;
    for (auto const& p : m) {
        auto ch = n->append_child();
        auto r = freal(p.first);
        ch << c4::yml::key(r);
        ch << freal(p.second);
    }
}

template <class Less, class Alloc>
void write(c4::yml::NodeRef* n, std::map<double, float, Less, Alloc> const& m)
{
    *n |= c4::yml::MAP;
    for (auto const& p : m) {
        auto ch = n->append_child();
        auto r = freal(p.first);
        ch << c4::yml::key(r);
        ch << freal(p.second);
    }
}

template <class K, class Less, class Alloc>
void write(c4::yml::NodeRef* n, std::map<K, double, Less, Alloc> const& m)
{
    *n |= c4::yml::MAP;
    for (auto const& p : m) {
        auto ch = n->append_child();
        ch << c4::yml::key(p.first);
        ch << freal(p.second);
    }
}

}  // namespace yml
}  // namespace c4
