# Copyright (c) 2023 Franka Robotics GmbH
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
#
# Derived from franka_example_controllers (https://github.com/frankaemika/franka_ros2);
# modified for cho_robot_project, see NOTICE.

cmake_minimum_required(VERSION 3.5)
project(cho_controller_franka)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cho_controller_common REQUIRED)
find_package(cho_controller_base REQUIRED)
# cho_controller_common/trajectory/motion_limits_params.hpp, compiled here.
find_package(joint_limits REQUIRED)
find_package(cho_vla_core REQUIRED)
find_package(cho_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
    # controllers
    src/base_controller.cpp
    src/ee_state_broadcaster.cpp
    src/task_space_ik_controller.cpp
    src/task_space_velocity_controller.cpp
    src/joint_space_velocity_controller.cpp
    src/gravity_compensation_controller.cpp
    src/gripper_controller.cpp
    src/joint_space_position_controller.cpp
    src/joint_space_impedance_controller.cpp
    src/task_space_impedance_controller.cpp
    src/operational_space_controller.cpp
    src/joint_space_qp_controller.cpp
    src/task_space_qp_controller.cpp
    src/vla_controller.cpp
    # action servers
    src/servers/gripper_action_server.cpp
    src/servers/vla_action_server.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
    controller_interface
    joint_limits
    franka_msgs
    hardware_interface
    pinocchio
    pluginlib
    rclcpp
    rclcpp_lifecycle
    std_srvs
    realtime_tools
    control_msgs
    cho_controller_common
    cho_controller_base
    cho_vla_core
    cho_interfaces
)

pluginlib_export_plugin_description_file(
        controller_interface cho_controller_franka.xml)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(controller_manager REQUIRED)
    # Loads the controllers through pluginlib, so it needs the package
    # installed (colcon test does) but does not link it.
    ament_add_gtest(test_reactivation test/test_reactivation.cpp TIMEOUT 300)
    ament_target_dependencies(test_reactivation
        ament_index_cpp cho_controller_base cho_interfaces controller_manager
        hardware_interface rclcpp rclcpp_action)
    # The gripper's result handoff alone: header-only, no ROS.
    ament_add_gtest(test_gripper_outcome test/test_gripper_outcome.cpp)
    target_include_directories(test_gripper_outcome PRIVATE include)
endif()

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_export_include_directories(
    include
)
ament_export_libraries(
    ${PROJECT_NAME}
)
ament_export_dependencies(
    controller_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    hardware_interface
)
ament_package()
