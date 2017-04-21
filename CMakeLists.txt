cmake_minimum_required (VERSION 2.6)
project(SimCoupler)

find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

if(PROTOBUF_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS protobuf-messages/state.proto protobuf-messages/vehicle.proto protobuf-messages/position.proto protobuf-messages/setup.proto)
endif()

if(BOOST_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

set (CMAKE_CXX_STANDARD 11)

# VREP
# FIXME adjust VREP_PATH if necessary
set(VREP_PATH "/usr/share/vrep/programming")
add_library(libvrep STATIC "${VREP_PATH}/remoteApi/extApi.c" "${VREP_PATH}/remoteApi/extApiPlatform.c")
set_target_properties(libvrep PROPERTIES COMPILE_FLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D__linux")
target_include_directories(libvrep PUBLIC /usr/share/vrep/programming/remoteApi)
target_include_directories(libvrep PUBLIC /usr/share/vrep/programming/include)

# SimCoupler
link_directories($ENV{SUMO_HOME}/src/utils/traci $ENV{SUMO_HOME}/src/foreign/tcpip $ENV{SUMO_HOME}/src/utils/common)
include_directories($ENV{SUMO_HOME}/src)
include_directories(${CMAKE_CURRENT_BINARY_DIR}) # protobuf *.o files
include_directories(inc)
AUX_SOURCE_DIRECTORY(src SRC_CC)
add_executable(main "${VREP_PATH}/remoteApi/extApi.c" "${VREP_PATH}/remoteApi/extApiPlatform.c" ${SRC_VREP} ${SRC_CC} ${PROTO_SRCS} ${PROTO_HDRS})
set_target_properties(main PROPERTIES COMPILE_FLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D__linux")
# includes
target_include_directories(main PUBLIC ${VREP_PATH}/remoteApi ${VREP_PATH}/include) # VREP

# -------
# linking
# -------
target_link_libraries(main libvrep) # VREP
target_link_libraries(main common traci tcpip protobuf) # SUMO
target_link_libraries(main ${PROTOBUF_LIBRARIES}) # Protobuf
target_link_libraries(main ${Boost_LIBRARIES}) # Boost
