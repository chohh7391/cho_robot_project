# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(cho_controller_openarm)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cho_controller_common REQUIRED)
find_package(cho_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/base_controller.cpp
    src/ee_state_broadcaster.cpp
    src/joint_space_impedance_controller.cpp
    src/joint_space_position_controller.cpp
    src/joint_space_velocity_controller.cpp
    src/servers/joint_space_action_server.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
    ament_index_cpp
    controller_interface
    geometry_msgs
    hardware_interface
    pinocchio
    pluginlib
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    realtime_tools
    control_msgs
    cho_controller_common
    cho_interfaces
)

pluginlib_export_plugin_description_file(controller_interface cho_controller_openarm.xml)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(controller_interface pluginlib rclcpp rclcpp_lifecycle hardware_interface)
ament_package()
