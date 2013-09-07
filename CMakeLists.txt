cmake_minimum_required(VERSION 2.8.3)
project(enu)

find_package(catkin REQUIRED COMPONENTS rosconsole roscpp geometry_msgs sensor_msgs nav_msgs swiftnav)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES swiftnav enu_ros
  CATKIN_DEPENDS rosconsole roscpp geometry_msgs sensor_msgs nav_msgs swiftnav
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare cpp library
add_library(enu_ros src/enu_ros.cpp)
target_link_libraries(enu_ros swiftnav ${catkin_LIBRARIES})

add_executable(enu_from_fix src/from_fix.cpp)
target_link_libraries(enu_from_fix enu_ros ${catkin_LIBRARIES})
set_target_properties(enu_from_fix PROPERTIES OUTPUT_NAME from_fix PREFIX "")

add_executable(enu_to_fix src/to_fix.cpp)
target_link_libraries(enu_to_fix enu_ros ${catkin_LIBRARIES})
set_target_properties(enu_to_fix PROPERTIES OUTPUT_NAME to_fix PREFIX "")

#############
## Install ##
#############

# Mark node executables for installation.
install(TARGETS enu_from_fix enu_to_fix
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install enu_ros library.
install(TARGETS enu_ros
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install enu_ros header.
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Docs ##
##########

add_custom_target (${PROJECT_NAME}_doc 
  rosdoc_lite -o ${CATKIN_DEVEL_PREFIX}/doc/${PROJECT_NAME} ${CATKIN_DEVEL_PREFIX}/../src/${PROJECT_NAME}
)

FILE(GLOB LINT_SRCS src/*.cpp include/enu/*.h)
add_custom_target(${PROJECT_NAME}_cpplint
  COMMAND cpplint --filter=-whitespace/line_length ${LINT_SRCS})
