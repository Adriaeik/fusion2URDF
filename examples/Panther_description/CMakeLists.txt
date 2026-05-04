cmake_minimum_required(VERSION 3.8)
project(Panther_description)

find_package(ament_cmake REQUIRED)

# Install package data
install(DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
