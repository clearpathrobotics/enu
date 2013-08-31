cmake_minimum_required(VERSION 2.8.3)
project(enu)

find_package(catkin REQUIRED COMPONENTS rosconsole roscpp geometry_msgs sensor_msgs nav_msgs)

catkin_package(
  INCLUDE_DIRS include src/libswiftnav/include
  LIBRARIES swiftnav
  CATKIN_DEPENDS rosconsole roscpp geometry_msgs sensor_msgs nav_msgs
)

###########
## Build ##
###########

include_directories(include src/libswiftnav/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Build the swiftnav library used for coordinate transforms.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} src/libswiftnav/cmake/)
add_subdirectory(src/libswiftnav)

## Declare cpp library
add_library(enu_swiftnav src/swiftnav.cpp)
#add_dependencies(GraftOdometryTopic ${PROJECT_NAME}_gencpp)

## Declare cpp executable
add_executable(enu_from_fix src/from_fix.cpp)
target_link_libraries(enu_from_fix enu_swiftnav swiftnav ${catkin_LIBRARIES})
set_target_properties(enu_from_fix PROPERTIES OUTPUT_NAME from_fix PREFIX "")

add_executable(enu_to_fix src/to_fix.cpp)
target_link_libraries(enu_to_fix swiftnav ${catkin_LIBRARIES})
set_target_properties(enu_to_fix PROPERTIES OUTPUT_NAME to_fix PREFIX "")

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS enu_from_fix enu_to_fix
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
