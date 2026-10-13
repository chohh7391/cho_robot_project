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
project(cho_openarm_mit_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/mit_protocol.cpp src/mit_limiter.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} trajectory_msgs)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES DESIGN.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Build-file and manifest lint; clang-format only on the limiter (the MuJoCo
  # safety boundary, moved here formatted). The rest predates any formatter.
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
  ament_clang_format(
    include/${PROJECT_NAME}/mit_limiter.hpp src/mit_limiter.cpp
    test/test_mit_limiter.cpp test/test_limiter_vs_consumer.cpp)
  find_package(cho_description_openarm REQUIRED)
  # The description's installed share directory (symlinked to the source
  # under --symlink-install), not a path back into the source tree.
  set(OPENARM_DESCRIPTION_SHARE "${cho_description_openarm_DIR}/..")
  ament_add_gtest(test_safety_profile test/test_safety_profile.cpp)
  target_link_libraries(test_safety_profile ${PROJECT_NAME})
  ament_add_gtest(test_arm_consumer test/test_arm_consumer.cpp)
  target_link_libraries(test_arm_consumer ${PROJECT_NAME})
  ament_add_gtest(test_switch_gate test/test_switch_gate.cpp)
  target_link_libraries(test_switch_gate ${PROJECT_NAME})
  ament_add_gtest(test_mit_limiter test/test_mit_limiter.cpp)
  target_link_libraries(test_mit_limiter ${PROJECT_NAME})
  # The MuJoCo limiter against the real consumer, on the same SEEDING commits.
  ament_add_gtest(test_limiter_vs_consumer test/test_limiter_vs_consumer.cpp)
  target_link_libraries(test_limiter_vs_consumer ${PROJECT_NAME})
  foreach(profile_test test_safety_profile test_mit_limiter test_limiter_vs_consumer)
    target_compile_definitions(${profile_test} PRIVATE
      OPENARM_SAFETY_PROFILE_SOURCE="${OPENARM_DESCRIPTION_SHARE}/config/mit_safety_profiles_v1.yaml")
  endforeach()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(trajectory_msgs yaml-cpp)
ament_package()
