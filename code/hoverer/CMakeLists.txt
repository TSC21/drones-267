cmake_minimum_required(VERSION 2.8.3)
project(hoverer)

find_package(catkin REQUIRED COMPONENTS
  	genmsg
    roscpp
    std_msgs
    roscopter
)

find_package(Boost REQUIRED COMPONENTS
    program_options
)


## Declare ROS messages, services and actions

## Generate added messages and services
# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

## catkin specific configuration
catkin_package(
)

## Build

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})


add_executable(Hoverer
    Hoverer.cpp
    Controller.h
    Controller.cpp
    HovererStates.h
    HovererStates.cpp
    QuadcopterState.h
    QuadcopterState.cpp
)

target_link_libraries(Hoverer ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS Hoverer
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set( CMAKE_BUILD_TYPE Debug )
