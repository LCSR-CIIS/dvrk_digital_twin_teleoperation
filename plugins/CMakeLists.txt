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

cmake_minimum_required(VERSION 3.5)
project(volumetric_plugin)

# Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(Boost REQUIRED COMPONENTS program_options filesystem)
find_package(AMBF REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

set (ROS1_depends ambf_server)
find_package (catkin QUIET
  COMPONENTS ${ROS1_depends})

if (catkin_FOUND)
    message ("Found ROS 1")
    set (AMBF_ROS_VERSION 1)
    include_directories(include ${catkin_INCLUDE_DIRS})

    catkin_package(
      INCLUDE_DIRS .
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS
    )

else(catkin_FOUND)
  # look for ROS2
  set (AMBF_ROS_VERSION 2)
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
      set (AMBF_ROS_VERSION 2)
      message ("Found ROS 2")
      set (ROS2_depends
        rclcpp geometry_msgs sensor_msgs
        ambf_msgs ambf_server)

      foreach (_dep ${ROS2_depends})
        find_package (${_dep} REQUIRED)
      endforeach()

      # include_directories(include)
  endif (ament_cmake_FOUND)

endif(catkin_FOUND)

include_directories(${AMBF_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

# link_directories(${AMBF_LIBRARY_DIRS})
add_definitions(${AMBF_DEFINITIONS})

# Plugin: ar_ros_plugin
add_library(ar_ros_plugin SHARED
  ar_ros_plugin/ar_ros_plugin.cpp
  ar_ros_plugin/ar_ros_plugin.hpp
)
ament_target_dependencies (ar_ros_plugin cv_bridge image_transport)

# Plugin: projection_override
add_library(camera_projection_override SHARED
    camera_projection_override/projection_override.cpp
    camera_projection_override/projection_override.h
)

# # Object Plugins
# add_library(ambf_crtk_object_plugin SHARED
#     src/utils/afCRTKInterface.cpp
#     src/utils/afCRTKInterface.h
#     src/utils/CRTK_base_plugin.cpp
#     src/utils/CRTK_base_plugin.h   
#     src/object_plugin/CRTK_object_plugin.cpp
#     src/object_plugin/CRTK_object_plugin.h
#     )

set (ROS_AR_PLUGINS ar_ros_plugin camera_projection_override)
# set (ROS_AR_PLUGINS ambf_crtk_simulator_plugin ambf_crtk_model_plugin ambf_crtk_object_plugin )
if (AMBF_ROS_VERSION EQUAL 1)

  foreach (_plugin ${ROS_AR_PLUGINS})
    add_dependencies (${_plugin} ${catkin_EXPORTED_TARGETS})
    target_link_libraries (${_plugin} ${catkin_LIBRARIES} ${AMBF_LIBRARIES})
  endforeach ()

else ()

  install (
    TARGETS ${ROS_AR_PLUGINS}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    )

  foreach (_plugin ${ROS_AR_PLUGINS})
    ament_target_dependencies (${_plugin} ambf_server ${ROS2_depends})
  endforeach ()
  ament_package ()

endif()

# # Plugin: peg_gripper_plugin
# add_library(peg_gripper_plugin SHARED
#   peg_gripper_plugin/peg_gripper_plugin.cpp
#   peg_gripper_plugin/peg_gripper_plugin.h
# )
# target_link_libraries(peg_gripper_plugin ${AMBF_LIBRARIES})

# # Plugin: keyboard_shortcuts_plugin
# add_library(keyboard_shortcuts_plugin SHARED
#   keyboard_shortcuts_plugin/keyboard_shortcuts.cpp
#   keyboard_shortcuts_plugin/keyboard_shortcuts.h
# )
# target_link_libraries(keyboard_shortcuts_plugin ${AMBF_LIBRARIES})
