cmake_minimum_required(VERSION 3.5)
project(cho_description_fr5)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf xml
  DESTINATION share/${PROJECT_NAME}
)

# USD assets for Isaac Sim are generated and intentionally ignored by git. The
# FR5 Isaac launch reports the exact conversion command when the asset is
# missing. This conditional lets a fresh clone build before conversion. If USD
# is generated in this source directory after the package was already
# configured, rebuild with `--cmake-force-configure` so CMake adds the install
# rule (generating directly in the installed package share needs no rebuild).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/usd")
  install(DIRECTORY
    usd
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
