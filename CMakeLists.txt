cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(DMR_VisionServer)

option(BUILD_TESTS "Build tests." OFF)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

find_package(PCL 1.8.0 REQUIRED)
find_package(RSSDK REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${RSSDK_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_library(real_sense
  src/real_sense/real_sense_device_manager.cpp
  src/real_sense_grabber.cpp
  src/io_exception.cpp
)

add_executable(DMR_VisionServer
  src/dmr_visionserver.cpp
)
target_link_libraries(DMR_VisionServer
  real_sense
  ${PCL_LIBRARIES}
  ${RSSDK_LIBRARIES}
)

