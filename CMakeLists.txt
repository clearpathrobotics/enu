cmake_minimum_required(VERSION 2.8.3)
project(enu)

find_package(catkin REQUIRED COMPONENTS rosconsole roscpp geometry_msgs sensor_msgs nav_msgs)

catkin_package(
  INCLUDE_DIRS src/libswiftnav/include
  LIBRARIES swiftnav
  CATKIN_DEPENDS rosconsole roscpp geometry_msgs sensor_msgs nav_msgs
)

###########
## Build ##
###########

include_directories(src/libswiftnav/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Build the swiftnav library used for coordinate transforms.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} src/libswiftnav/cmake/)
add_subdirectory(src/libswiftnav)

## Declare cpp library
#add_library(GraftOdometryTopic src/GraftOdometryTopic.cpp)
#add_dependencies(GraftOdometryTopic ${PROJECT_NAME}_gencpp)

## Declare cpp executable
add_executable(enu_from_fix src/from_fix.cpp)
target_link_libraries(enu_from_fix swiftnav ${catkin_LIBRARIES})

add_executable(enu_to_fix src/to_fix.cpp)
target_link_libraries(enu_to_fix swiftnav ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS enu_from_fix enu_to_fix
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
