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
project(cho_controller_base)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(cho_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

# The one piece that is compiled: the process-wide HeldCommandLedger
# (held_command.hpp). It has to be a shared library of its own so every
# controller plugin, each dlopen()ed separately, shares one instance.
add_library(${PROJECT_NAME}_ledger SHARED src/held_command_ledger.cpp)
target_include_directories(${PROJECT_NAME}_ledger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_ledger PUBLIC Eigen3::Eigen)

# Otherwise header-only: the servers are templates over the controller's state
# and trajectory types, instantiated in each controller package.
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} INTERFACE
  ${PROJECT_NAME}_ledger
  rclcpp::rclcpp rclcpp_action::rclcpp_action rclcpp_lifecycle::rclcpp_lifecycle
  Eigen3::Eigen pinocchio::pinocchio)
ament_target_dependencies(${PROJECT_NAME} INTERFACE cho_interfaces sensor_msgs)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_ledger EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
  ament_add_gtest(test_goal_phase_action_server test/test_goal_phase_action_server.cpp TIMEOUT 120)
  target_link_libraries(test_goal_phase_action_server ${PROJECT_NAME})
  ament_add_gtest(test_motion_servers test/test_motion_servers.cpp TIMEOUT 120)
  target_link_libraries(test_motion_servers ${PROJECT_NAME})
  ament_add_gtest(test_helpers test/test_helpers.cpp)
  target_link_libraries(test_helpers ${PROJECT_NAME})
  ament_add_gtest(test_dls_no_malloc test/test_dls_no_malloc.cpp)
  target_link_libraries(test_dls_no_malloc ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(cho_interfaces Eigen3 pinocchio rclcpp rclcpp_action rclcpp_lifecycle sensor_msgs)
ament_package()
