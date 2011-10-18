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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(flirtlib_ros
                     src/conversions.cpp)

rosbuild_add_executable(ransac_test src/RansacLoopClosureTest.cpp)
target_link_libraries(ransac_test feature sensorstream sensors utils)

rosbuild_add_executable(flirtlib_ros_test src/flirtlib_ros_test.cpp)
target_link_libraries(flirtlib_ros_test feature sensorstream sensors utils)
target_link_libraries(flirtlib_ros_test grid_utils)
target_link_libraries(flirtlib_ros_test flirtlib_ros)

rosbuild_add_executable(markers src/interactive_markers.cpp)

rosbuild_add_executable(simulate_scans src/simulate_scans.cpp)
target_link_libraries(simulate_scans grid_utils)