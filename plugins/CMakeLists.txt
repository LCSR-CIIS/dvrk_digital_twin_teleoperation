# Software License Agreement (BSD License)
# Copyright (c) 2019-2021, AMBF.
# (https://github.com/WPI-AIM/ambf)
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
#
# * Neither the name of authors nor the names of its contributors may
# be used to endorse or promote products derived from this software
# without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# $Author: Juan Antonio Barragan $
# $Date: 08/29/2024 $
# $Rev:  $

cmake_minimum_required(VERSION 3.1)
project(volumetric_plugin)

set(CMAKE_CXX_STANDARD 11)

find_package(AMBF)
find_package(Boost COMPONENTS program_options filesystem)

include_directories(${AMBF_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

link_directories(${AMBF_LIBRARY_DIRS})

add_definitions(${AMBF_DEFINITIONS})

message("---> LIBS " ${catkin_INCLUDE_DIRS})

# # Camera overwrite plugin
add_library(camera_projection_override SHARED
    camera_projection_override/projection_override.cpp
    camera_projection_override/projection_override.h)

target_link_libraries(camera_projection_override ${AMBF_LIBRARIES})
set_property(TARGET camera_projection_override PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# # AR plugin
add_library(ar_ros_plugin SHARED
    ar_ros_plugin/ar_ros_plugin.cpp
    ar_ros_plugin/ar_ros_plugin.h)

target_link_libraries(ar_ros_plugin ${AMBF_LIBRARIES})
set_property(TARGET ar_ros_plugin PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# # Test rendering override
add_library(test_rendering_override SHARED
    test_rendering_override/test_rendering_override.cpp
    test_rendering_override/test_rendering_override.h)

target_link_libraries(test_rendering_override ${AMBF_LIBRARIES})
set_property(TARGET test_rendering_override PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# # Peg Gripper plugin
add_library(peg_gripper_plugin SHARED
    peg_gripper_plugin/peg_gripper_plugin.cpp
    peg_gripper_plugin/peg_gripper_plugin.h)

target_link_libraries(peg_gripper_plugin ${AMBF_LIBRARIES})
set_property(TARGET peg_gripper_plugin PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# # Keyboard shorcuts plugin
add_library(keyboard_shortcuts_plugin SHARED 
    keyboard_shortcuts_plugin/keyboard_shortcuts.cpp 
    keyboard_shortcuts_plugin/keyboard_shortcuts.h)
target_link_libraries(keyboard_shortcuts_plugin ${AMBF_LIBRARIES})
set_property(TARGET keyboard_shortcuts_plugin PROPERTY POSITION_INDEPENDENT_CODE TRUE)