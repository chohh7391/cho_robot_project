# Copyright 2025 Enactic, Inc.
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
project(cho_description_openarm)
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(${PROJECT_NAME}_urdf_tests test/urdf_tests.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(${PROJECT_NAME}_mit_contract_tests test/mit_contract_tests.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(${PROJECT_NAME}_mit_protocol_model_tests test/mit_protocol_model_tests.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

install(DIRECTORY
  assets config robots xml launch rviz test
  DESTINATION share/${PROJECT_NAME}/
)

# USD assets for Isaac Sim. Generated and large, so they are not tracked in git
# (see .gitignore). Guarded so a clone that has not run the converter builds.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/usd")
  install(DIRECTORY
    usd
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_package()
