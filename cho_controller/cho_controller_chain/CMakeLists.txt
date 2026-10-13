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

cmake_minimum_required(VERSION 3.8)
project(cho_controller_chain)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(cho_controller_base REQUIRED)
find_package(cho_controller_common REQUIRED)
find_package(cho_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(hardware_interface REQUIRED)
# cho_controller_common/trajectory/motion_limits_params.hpp, compiled here.
find_package(joint_limits REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

# Each controller's parameters, declared and range-checked from YAML (CLAUDE.md,
# "Controllers"); the checks that relate parameters run in on_configure.
generate_parameter_library(chain_joint_impedance_law_parameters
  src/joint_impedance_law_parameters.yaml)
generate_parameter_library(chain_joint_space_generator_parameters
  src/joint_space_generator_parameters.yaml)
generate_parameter_library(chain_task_space_generator_parameters
  src/task_space_generator_parameters.yaml)

# One library for the law, its ROS-free math and the two generators. The
# harness tests load the controllers through pluginlib and do not link it.
add_library(${PROJECT_NAME} SHARED
  src/joint_impedance.cpp
  src/robot_model.cpp
  src/joint_impedance_law.cpp
  src/reference_generator.cpp
  src/joint_space_generator.cpp
  src/task_space_generator.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  cho_controller_base
  cho_controller_common
  cho_interfaces
  control_msgs
  controller_interface
  hardware_interface
  joint_limits
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
)
# pinocchio's imported target, not ament_target_dependencies: it carries the
# compile definitions and the URDF parser library (see cho_controller_common).
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  pinocchio::pinocchio
  chain_joint_impedance_law_parameters
  chain_joint_space_generator_parameters
  chain_task_space_generator_parameters
)

pluginlib_export_plugin_description_file(controller_interface cho_controller_chain.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(controller_manager REQUIRED)
  ament_lint_cmake()
  ament_xmllint()

  # The law's torque and the reduced model, no ROS graph: links the library;
  # the FR3 URDF comes from cho_description_franka at run time.
  ament_add_gtest(test_joint_impedance_torque test/test_joint_impedance_torque.cpp)
  target_link_libraries(test_joint_impedance_torque ${PROJECT_NAME} pinocchio::pinocchio)
  ament_target_dependencies(test_joint_impedance_torque ament_index_cpp)

  # The law and the generators through the real controller_manager lifecycle on
  # a mock FR3 (cho_controller_base's harness). They load the controllers
  # through pluginlib, so they need the package installed (colcon test does)
  # but do not link it: a second copy of the plugin library in the process
  # confuses class_loader. impedance_law_probe.hpp is header-only.
  foreach(harness_test test_chain_switch test_chain_generators)
    ament_add_gtest(${harness_test} test/${harness_test}.cpp TIMEOUT 300)
    target_include_directories(${harness_test} PRIVATE include)
    target_link_libraries(${harness_test} pinocchio::pinocchio)
    ament_target_dependencies(${harness_test}
      ament_index_cpp cho_controller_base cho_interfaces controller_manager
      hardware_interface rclcpp rclcpp_action)
  endforeach()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  cho_controller_base
  cho_controller_common
  cho_interfaces
  control_msgs
  controller_interface
  Eigen3
  generate_parameter_library
  hardware_interface
  joint_limits
  pinocchio
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
)
ament_package()
