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
   Desc: Custom exception and error handler for ryml
*/

#pragma once

#include <stdexcept>
#include <c4/format.hpp>
#include <ryml.hpp>

namespace moveit_serialization {

/// custom ryml runtime_error exception
class yaml_error : public std::runtime_error
{
public:
    explicit yaml_error(const std::string& what_arg) : runtime_error(what_arg){};
    explicit yaml_error(const char* what_arg) : runtime_error(what_arg){};
};

struct ErrorHandler
{
    // this will be called on error
    void on_error(const char* msg, size_t len, ryml::Location loc)
    {
        throw yaml_error(ryml::formatrs<std::string>("{}:{}:{} ({}B): ERROR: {}", loc.name, loc.line, loc.col,
                                                     loc.offset, ryml::csubstr(msg, len)));
    }

    // bridge
    ryml::Callbacks callbacks()
    {
        return ryml::Callbacks(this, nullptr, nullptr, ErrorHandler::s_error);
    }
    static void s_error(const char* msg, size_t len, ryml::Location loc, void* this_)
    {
        return ((ErrorHandler*)this_)->on_error(msg, len, loc);
    }

    ErrorHandler() : defaults(ryml::get_callbacks())
    {}
    ryml::Callbacks defaults;
};

}  // namespace moveit_serialization
