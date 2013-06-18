cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(src/libswiftnav/include)
rosbuild_add_boost_directories()

rosbuild_add_executable(from_fix src/from_fix.cpp)
target_link_libraries(from_fix swiftnav)

rosbuild_add_executable(to_fix src/to_fix.cpp)
target_link_libraries(to_fix swiftnav)

# Build the swiftnav library used for coordinate transforms.
add_subdirectory(src/libswiftnav)
