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
   Desc: ryml custom node manipulation
*/

#pragma once

#include <c4/yml/node.hpp>
#include <moveit_serialization/ryml/error_handler.h>

#include <tuple>

namespace c4 {
namespace yml {

/* Given */
bool getNodeFromKeyChainVal(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target,
                            c4::yml::NodeRef& scalar);

/**
 * c++14 equivalent of 'if constexpr' (c++17)
 */
template <class Integral, Integral N>
auto dispatch(std::integral_constant<Integral, N>)
{
    return [](auto&&... args) { return std::get<N>(std::forward_as_tuple(decltype(args)(args)...)); };
}
template <std::size_t N>
auto dispatch()
{
    return dispatch(std::integral_constant<std::size_t, N>{});
}

/**
 * Comparison between 2 ryml::NodeRef
 *
 * Given some types, the nodes are decoded and compared.
 */
template <typename... Ts>
struct scalar_compare
{
    bool equality(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target)
    {
        return _impl_equality<0>(source, target);
    }

    bool non_equality(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target)
    {
        return !_impl_equality<0>(source, target);
    }

    bool lower_then(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target)
    {
        return _impl_lower_then<0>(source, target);
    }

    bool greater_then(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target)
    {
        return _impl_greater_then<0>(source, target);
    }

    bool lower_equal(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target)
    {
        return !_impl_greater_then<0>(source, target);
    }

    bool greater_equal(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target)
    {
        return !_impl_lower_then<0>(source, target);
    }

private:
    using TypeTuple = std::tuple<Ts...>;
    constexpr static std::size_t TUPLE_SIZE = std::tuple_size<TypeTuple>::value;

    template <std::size_t N>
    bool _impl_equality(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target)
    {
        return dispatch(std::integral_constant<bool, N == TUPLE_SIZE>{})(
            // 0, aka false branch:
            [&](auto Nval) {
                using SelectedType = std::tuple_element_t<Nval, TypeTuple>;

                try {
                    SelectedType src;
                    SelectedType trg;

                    source >> src;
                    target >> trg;

                    return src == trg;
                }
                catch (moveit_serialization::yaml_error& e) {
                    return _impl_equality<Nval + 1>(source, target);
                }
            },
            // 1, aka true branch:
            [](auto&&) { return false; })(std::integral_constant<std::size_t, N>{});
    }

    template <std::size_t N>
    bool _impl_greater_then(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target)
    {
        return dispatch(std::integral_constant<bool, N == TUPLE_SIZE>{})(
            // 0, aka false branch:
            [&](auto Nval) {
                using SelectedType = std::tuple_element_t<Nval, TypeTuple>;

                try {
                    SelectedType src;
                    SelectedType trg;

                    source >> src;
                    target >> trg;
                    return src > trg;
                }
                catch (moveit_serialization::yaml_error& e) {
                    return _impl_greater_then<Nval + 1>(source, target);
                }
            },
            // 1, aka true branch:
            [](auto&&) { return false; })(std::integral_constant<std::size_t, N>{});
    }

    template <std::size_t N>
    bool _impl_lower_then(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target)
    {
        return dispatch(std::integral_constant<bool, N == TUPLE_SIZE>{})(
            // 0, aka false branch:
            [&](auto Nval) {
                using SelectedType = std::tuple_element_t<Nval, TypeTuple>;

                try {
                    SelectedType src;
                    SelectedType trg;

                    source >> src;
                    target >> trg;
                    return src < trg;
                }
                catch (moveit_serialization::yaml_error& e) {
                    return _impl_lower_then<Nval + 1>(source, target);
                }
            },
            // 1, aka true branch:
            [](auto&&) { return false; })(std::integral_constant<std::size_t, N>{});
    }
};

template <class... Args>
bool scalar_compare_eq(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target);

template <class T, class... Args>
bool helper_compare_eq(const c4::yml::ConstNodeRef& source, const c4::yml::ConstNodeRef& target)
{
    try {
        T src;
        T trg;

        source >> src;
        target >> trg;
        return src == trg;
    }
    catch (moveit_serialization::yaml_error& e) {
        return scalar_compare_eq<Args...>(source, target);
    }
}

}  // namespace yml
}  // namespace c4
