cmake_minimum_required(VERSION 3.8)
project(cho_moveit_fr5)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant UPSTREAM.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
