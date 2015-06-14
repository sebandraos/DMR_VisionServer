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

#OSCPACK Zone

IF(WIN32)
 set(IpSystemTypePath include/ip/win32)
 set(LIBS ${LIBS} Ws2_32 winmm)
ELSE(WIN32)
 set(IpSystemTypePath include/ip/posix)
ENDIF(WIN32)

ADD_LIBRARY(oscpack 

include/ip/IpEndpointName.h
include/ip/IpEndpointName.cpp

include/ip/NetworkingUtils.h
${IpSystemTypePath}/NetworkingUtils.cpp

include/ip/UdpSocket.h
${IpSystemTypePath}/UdpSocket.cpp

include/ip/PacketListener.h
include/ip/TimerListener.h

include/osc/OscTypes.h
include/osc/OscTypes.cpp 
include/osc/OscHostEndianness.h
include/osc/OscException.h
include/osc/OscPacketListener.h
include/osc/MessageMappingOscPacketListener.h
include/osc/OscReceivedElements.h
include/osc/OscReceivedElements.cpp
include/osc/OscPrintReceivedElements.h
include/osc/OscPrintReceivedElements.cpp
include/osc/OscOutboundPacketStream.h
include/osc/OscOutboundPacketStream.cpp
)

add_executable(DMR_VisionServer
  src/dmr_visionserver.cpp
)
target_link_libraries(DMR_VisionServer
  real_sense
  ${PCL_LIBRARIES}
  ${RSSDK_LIBRARIES}
  oscpack 
  ${LIBS}
)

