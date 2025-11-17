cmake_minimum_required(VERSION 3.8)

project(d2_obstacle_viewer)
set(PROJECT_DIR d2/obstacle_viewer)
set(PROJECT_NAMESPACE d2::obstacle_viewer)

# find package
if("$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
else()
endif()

# get source files without ros
file(GLOB_RECURSE ${PROJECT_NAME}_src_files
  LIST_DIRECTORIES false
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/**.cpp")

# remove ROS2 src if ROS_VERSION is not 2
if(NOT "$ENV{ROS_VERSION}" STREQUAL "2")
  file(GLOB_RECURSE ${PROJECT_NAME}_ros2_src_files
  LIST_DIRECTORIES false
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/${PROJECT_DIR}/ros2/*.cpp")
  list(REMOVE_ITEM ${PROJECT_NAME}_src_files
    ${${PROJECT_NAME}_ros2_src_files})
endif()

# create library
list(LENGTH ${PROJECT_NAME}_src_files src_files_count)
if(src_files_count GREATER 0)
  if("$ENV{ROS_VERSION}" STREQUAL "2")
    # ROS2 library
    ament_auto_add_library(${PROJECT_NAME}_lib SHARED "${${PROJECT_NAME}_src_files}")
  else()
    # non-ROS library
    find_package(ament_cmake REQUIRED)
  endif()
else()
  # create header-only library if no source files
  add_library(${PROJECT_NAME}_lib INTERFACE)
  target_include_directories(${PROJECT_NAME}_lib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endif()

# set target properties
target_compile_features(${PROJECT_NAME}_lib PUBLIC c_std_99 cxx_std_17)
target_compile_options(${PROJECT_NAME}_lib PRIVATE -Wall -Wextra -Wpedantic -O2)

# install directories
install(DIRECTORY
  # config
  # launch
  DESTINATION share/${PROJECT_NAME}/)

# register components(ROS2)
if("$ENV{ROS_VERSION}" STREQUAL "2")
rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "${PROJECT_NAMESPACE}::ObstacleViewerNode"
  EXECUTABLE "obstacle_viewer_node")
endif()

# test
if(BUILD_TESTING)
  if("$ENV{ROS_VERSION}" STREQUAL "2")
    ament_auto_find_test_dependencies()
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

# package(ROS2)
if("$ENV{ROS_VERSION}" STREQUAL "2")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
endif()
