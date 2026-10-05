# Copyright (c) 2021 PickNik, Inc.
# Copyright 2026 Hyunho Cho
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holders nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Derived from ur_description (https://github.com/UniversalRobots/Universal_Robots_ROS2_Description); see NOTICE.

cmake_minimum_required(VERSION 3.5)
project(cho_description_ur)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf test xml
  DESTINATION share/${PROJECT_NAME}
)

# USD assets for Isaac Sim. Generated (and large), so they are not tracked in
# git -- see cho_description_franka/usd/README.md for the conversion command.
# Guarded so a fresh clone that has not run the converter yet still builds.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/usd")
  install(DIRECTORY
    usd
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(ur_urdf_xacro test/test_ur_urdf_xacro.py)
  ament_add_pytest_test(view_ur_launch test/test_view_ur_launch.py)
endif()

ament_package()
