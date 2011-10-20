cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

rosbuild_add_library(flirtlib_ros
                     src/conversions.cpp)

rosbuild_add_executable(ransac_test src/RansacLoopClosureTest.cpp)
target_link_libraries(ransac_test feature sensorstream sensors utils)

rosbuild_add_executable(flirtlib_ros_test src/flirtlib_ros_test.cpp)
target_link_libraries(flirtlib_ros_test feature sensorstream sensors utils)
target_link_libraries(flirtlib_ros_test flirtlib_ros)

rosbuild_add_executable(markers src/interactive_markers.cpp)

rosbuild_add_executable(simulate_scans src/simulate_scans.cpp)
target_link_libraries(simulate_scans grid_utils)

rosbuild_add_executable(place_rec_test src/place_rec_test.cpp)
target_link_libraries(place_rec_test grid_utils)
target_link_libraries(place_rec_test flirtlib_ros)

rosbuild_add_executable(startup_loc src/startup_loc.cpp)
target_link_libraries(startup_loc grid_utils)
target_link_libraries(startup_loc flirtlib_ros)