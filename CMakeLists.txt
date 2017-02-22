cmake_minimum_required (VERSION 2.6)
project(SimCoupler)

find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

if(PROTOBUF_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS protobuf-messages/situation.proto protobuf-messages/vehicle.proto protobuf-messages/position.proto protobuf-messages/track.proto)
endif()

if(BOOST_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

include_directories($ENV{SUMO_HOME}/src)
link_directories($ENV{SUMO_HOME}/src/utils/traci $ENV{SUMO_HOME}/src/foreign/tcpip $ENV{SUMO_HOME}/src/utils/common)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_executable(main src/main.cc ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(main common traci tcpip protobuf)
target_link_libraries(main ${PROTOBUF_LIBRARIES})
target_link_libraries(main ${Boost_LIBRARIES})
