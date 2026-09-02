cmake_minimum_required(VERSION 3.8)
project(cho_moveit_common)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(moveit_action_bridge_tests
    test/test_moveit_action_bridge.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(static_scene_parameter_tests
    test/test_publish_static_scene_parameters.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=223
  )
endif()

install(PROGRAMS
  scripts/moveit_action_bridge.py
  scripts/publish_static_scene.py
  scripts/wait_for_static_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
