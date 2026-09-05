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
project(cho_controller_gripper)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cho_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/gripper_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
    cho_interfaces
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    realtime_tools
    std_msgs
)

pluginlib_export_plugin_description_file(controller_interface cho_controller_gripper.xml)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(controller_manager REQUIRED)

    ament_add_gtest(test_width_mapping test/test_width_mapping.cpp)
    target_include_directories(test_width_mapping PRIVATE include)

    ament_add_gtest(test_gripper_controller test/test_gripper_controller.cpp)
    target_link_libraries(test_gripper_controller ${PROJECT_NAME})
    ament_target_dependencies(test_gripper_controller
        cho_interfaces controller_interface controller_manager hardware_interface
        rclcpp rclcpp_action rclcpp_lifecycle std_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(controller_interface pluginlib rclcpp rclcpp_lifecycle hardware_interface)
ament_package()
