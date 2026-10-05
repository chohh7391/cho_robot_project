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
project(cho_controller_common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Release (-O3) unless told otherwise: the QP runs at 1 kHz, and an unset build
# type compiles it at -O0. Not -Ofast: it implies -ffinite-math-only, which folds
# isfinite()/isnan() to constants and lets the dense solver's `objective ==
# infinity` infeasibility test compile away, so an infeasible QP can come back
# OPTIMAL. In MuJoCo the task-space QP took the same ~10 us per cycle either way.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ruckig REQUIRED)
# Used only by the header-only trajectory/motion_limits_params.hpp, which the
# controller packages compile; exported below so they find it.
find_package(joint_limits REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Listed, not globbed: a glob is evaluated at configure time, so a source that
# is added or deleted is missed until someone forces a reconfigure.
add_library(${PROJECT_NAME} SHARED
  src/contact/contact_base.cpp
  src/formulation/contact_level.cpp
  src/formulation/inverse_dynamics_formulation_acc.cpp
  src/formulation/inverse_dynamics_formulation_base.cpp
  src/math/constraint_base.cpp
  src/math/constraint_bound.cpp
  src/math/constraint_equality.cpp
  src/math/constraint_inequality.cpp
  src/math/util.cpp
  src/robot/robot_wrapper.cpp
  src/solver/solver_HQP_base.cpp
  src/solver/solver_HQP_eiquadprog.cpp
  src/solver/solver_HQP_eiquadprog_fast.cpp
  src/solver/solver_HQP_factory.cpp
  src/tasks/task_base.cpp
  src/tasks/task_contact_force.cpp
  src/tasks/task_joint_posture.cpp
  src/tasks/task_motion.cpp
  src/tasks/task_se3_equality.cpp
  src/trajectory/point_to_point.cpp
  src/trajectory/trajectory_euclidian.cpp
  src/trajectory/trajectory_se3.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pinocchio
  eiquadprog
  Eigen3
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  eiquadprog::eiquadprog
  # pinocchio 3+/4.0: link the modern imported target explicitly.
  # ament_target_dependencies(pinocchio) does not propagate the target's
  # interface properties, so we would otherwise miss:
  #  - BOOST_MPL_LIMIT_LIST_SIZE=30 compile defs (JointCollectionDefault now
  #    has >20 boost::variant types), and
  #  - the split-out pinocchio_parsers lib holding the URDF parser symbols
  #    (pinocchio::urdf::buildModel), which caused an undefined-symbol crash
  #    at runtime.
  # pinocchio::pinocchio bundles default + parsers + collision (and headers).
  pinocchio::pinocchio
)
# No installed header includes Ruckig, but this file links with the plain
# signature, which makes every link transitive: ruckig is exported below so a
# consumer's find_package(cho_controller_common) defines ruckig::ruckig.
target_link_libraries(${PROJECT_NAME} ruckig::ruckig)

# Install
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  ruckig
  joint_limits
  pinocchio
  eiquadprog
  Eigen3
)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Build-file and manifest lint only: the C++ predates any formatter here.
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
  ament_add_gtest(test_point_to_point test/test_point_to_point.cpp)
  # The private planner header lives in src/.
  target_include_directories(test_point_to_point PRIVATE src)
  target_link_libraries(test_point_to_point ${PROJECT_NAME} pinocchio::pinocchio)
  # The control-thread half of a goal does not allocate (malloc interposed).
  ament_add_gtest(test_trajectory_no_alloc test/test_trajectory_no_alloc.cpp)
  target_link_libraries(test_trajectory_no_alloc ${PROJECT_NAME} pinocchio::pinocchio)
endif()

ament_package()
