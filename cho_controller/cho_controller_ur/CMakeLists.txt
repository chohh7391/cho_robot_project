cmake_minimum_required(VERSION 3.5)
project(cho_controller_ur)

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
find_package(hardware_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(cho_controller_common REQUIRED)
find_package(cho_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/base_controller.cpp
    src/joint_space_position_controller.cpp
    src/task_space_ik_controller.cpp
    src/servers/joint_space_action_server.cpp
    src/servers/task_space_action_server.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
    controller_interface
    hardware_interface
    pinocchio
    pluginlib
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    realtime_tools
    cho_controller_common
    cho_interfaces
)

pluginlib_export_plugin_description_file(
    controller_interface cho_controller_ur.xml)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(
    DIRECTORY include/
    DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
    controller_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    hardware_interface
)
ament_package()
