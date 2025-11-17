cmake_minimum_required(VERSION 3.8)

project(d2_costmap_bringup)

# find package
find_package(ament_cmake_auto REQUIRED)

# install directories
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
