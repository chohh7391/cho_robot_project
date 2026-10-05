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
project(cho_vla_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# This library's whole purpose is to reject non-finite policy output, and
# -ffinite-math-only (implied by -Ofast) folds std::isfinite() to true.
# Measured on g++ 11.4: an -Ofast build reports a NaN-carrying vector as
# all-finite, with no warning.
#
# This explicit -fno- wins over -Ofast regardless of which comes later on the
# command line (also measured), so it holds even if a parent scope or a toolchain
# file adds one. DO NOT DELETE IT: removing this line while -Ofast is in effect is
# the one combination that silently disables every rejection in this package, and
# test_finite_math_guard is the tripwire for exactly that case -- verified to fail
# with the line removed and to pass with it present.
add_compile_options(-fno-finite-math-only)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ruckig REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/types.cpp
  src/chunk_validator.cpp
  src/chunk_smoother.cpp
  src/action_buffer.cpp
  src/reference_history.cpp
  src/reference_limiter.cpp
  src/stream_watchdog.cpp
  src/gripper_dispatch.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} Eigen3 pinocchio)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen pinocchio::pinocchio ruckig::ruckig)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES DESIGN.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Build-file and manifest lint only: the C++ predates any formatter here.
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
  foreach(test_name
      test_finite_math_guard
      test_chunk_validator
      test_chunk_smoother
      test_action_buffer
      test_reference_limiter
      test_stream_watchdog
      test_gripper_dispatch
      test_reference_history)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    ament_target_dependencies(${test_name} Eigen3 pinocchio)
  endforeach()
endif()

ament_export_dependencies(Eigen3 pinocchio ruckig)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
