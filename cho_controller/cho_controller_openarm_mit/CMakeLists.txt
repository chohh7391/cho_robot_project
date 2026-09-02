cmake_minimum_required(VERSION 3.8)
project(cho_controller_openarm_mit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(cho_openarm_mit_core REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cho_interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/bimanual_fjt_controller.cpp
  src/single_arm_fjt_controller.cpp
  src/direct_mit_controller.cpp
  src/task_space_impedance_mit_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  cho_openarm_mit_core controller_interface control_msgs cho_interfaces hardware_interface
  pinocchio pluginlib rclcpp rclcpp_action rclcpp_lifecycle realtime_tools std_msgs
  std_srvs trajectory_msgs)
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)

pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES controller_plugins.xml DESTINATION share/${PROJECT_NAME})
install(FILES DESIGN.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(cho_description_openarm REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(controller_manager_msgs REQUIRED)
  find_package(cho_hardware_openarm_mit_test REQUIRED)

  ament_add_gtest(test_mit_protocol test/test_mit_protocol.cpp)
  target_link_libraries(test_mit_protocol ${PROJECT_NAME})
  ament_target_dependencies(test_mit_protocol controller_interface control_msgs hardware_interface pluginlib rclcpp)

  ament_add_gtest(test_direct_mit_controller test/test_direct_mit_controller.cpp)
  target_link_libraries(test_direct_mit_controller ${PROJECT_NAME})
  ament_target_dependencies(test_direct_mit_controller controller_interface hardware_interface rclcpp)

  ament_add_gtest(test_controller_manager_action test/test_controller_manager_action.cpp)
  target_link_libraries(test_controller_manager_action ${PROJECT_NAME})
  ament_target_dependencies(test_controller_manager_action
    cho_hardware_openarm_mit_test controller_manager controller_manager_msgs control_msgs
    hardware_interface rclcpp rclcpp_action)
  target_compile_definitions(test_controller_manager_action PRIVATE
    OPENARM_SAFETY_PROFILE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/../../cho_description/cho_description_openarm/config/mit_safety_profiles_v1.yaml")

  ament_add_gtest(test_direct_controller_manager test/test_direct_controller_manager.cpp)
  target_link_libraries(test_direct_controller_manager ${PROJECT_NAME})
  ament_target_dependencies(test_direct_controller_manager
    cho_hardware_openarm_mit_test controller_manager controller_manager_msgs hardware_interface
    rclcpp std_msgs std_srvs)
  target_compile_definitions(test_direct_controller_manager PRIVATE
    OPENARM_SAFETY_PROFILE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/../../cho_description/cho_description_openarm/config/mit_safety_profiles_v1.yaml")

  ament_add_gtest(test_impedance_action_controller test/test_impedance_action_controller.cpp)
  target_link_libraries(test_impedance_action_controller ${PROJECT_NAME})
  ament_target_dependencies(test_impedance_action_controller
    cho_hardware_openarm_mit_test controller_manager controller_manager_msgs hardware_interface
    rclcpp rclcpp_action cho_interfaces std_srvs)
  target_compile_definitions(test_impedance_action_controller PRIVATE
    OPENARM_SAFETY_PROFILE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/../../cho_description/cho_description_openarm/config/mit_safety_profiles_v1.yaml")

  foreach(arm_side left right)
    ament_add_gtest(test_single_${arm_side}_controller_manager_action
      test/test_single_controller_manager_action.cpp)
    target_link_libraries(test_single_${arm_side}_controller_manager_action ${PROJECT_NAME})
    ament_target_dependencies(test_single_${arm_side}_controller_manager_action
      cho_hardware_openarm_mit_test controller_manager controller_manager_msgs control_msgs
      hardware_interface rclcpp rclcpp_action)
    target_compile_definitions(test_single_${arm_side}_controller_manager_action PRIVATE
      TEST_ARM_SIDE="${arm_side}"
      OPENARM_SAFETY_PROFILE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/../../cho_description/cho_description_openarm/config/mit_safety_profiles_v1.yaml")
  endforeach()
endif()

ament_export_dependencies(
  cho_openarm_mit_core controller_interface control_msgs cho_interfaces hardware_interface
  pinocchio pluginlib rclcpp rclcpp_action rclcpp_lifecycle realtime_tools std_msgs
  std_srvs trajectory_msgs)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
