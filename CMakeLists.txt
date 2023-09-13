cmake_minimum_required(VERSION 3.14)
project(map_builder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(
  beam REQUIRED
  COMPONENTS
  utils
  mapping
  calibration
)

find_package(
  catkin REQUIRED 
  COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ManualCalibrationMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
)

## Declare a C++ library
add_library(
  ${PROJECT_NAME}
  src/MapBuilder.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
)
target_link_libraries(
  ${PROJECT_NAME}
    beam::utils
    beam::mapping
    beam::calibration
)

################ Executables ###################
add_executable(${PROJECT_NAME}_build_map
  src/build_map.cpp
)

target_include_directories(${PROJECT_NAME}_build_map
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_build_map
  ${PROJECT_NAME}
  beam::utils
  beam::mapping
)

add_executable(${PROJECT_NAME}_manual_calibration
  src/manual_calibration.cpp
)

target_include_directories(${PROJECT_NAME}_manual_calibration
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_manual_calibration
  ${PROJECT_NAME}
  beam::utils
  beam::mapping
)

add_dependencies(${PROJECT_NAME}_manual_calibration
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_loop_closed_paths_to_poses
  src/loop_closed_paths_to_poses.cpp
)

target_include_directories(${PROJECT_NAME}_loop_closed_paths_to_poses
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_loop_closed_paths_to_poses
  ${PROJECT_NAME}
  beam::utils
  beam::mapping
  beam::filtering
  beam::calibration
)

add_executable(${PROJECT_NAME}_fill_in_trajectory
  src/fill_in_trajectory.cpp
)

target_include_directories(${PROJECT_NAME}_fill_in_trajectory
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_fill_in_trajectory
  ${PROJECT_NAME}
  beam::utils
  beam::mapping
  beam::calibration
)

add_executable(${PROJECT_NAME}_path_messages_to_poses_files
  src/path_messages_to_pose_files.cpp
)

target_include_directories(${PROJECT_NAME}_path_messages_to_poses_files
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_path_messages_to_poses_files
  ${PROJECT_NAME}
  beam::utils
  beam::mapping
  beam::filtering
  beam::calibration
)

add_executable(${PROJECT_NAME}_bag_to_poses_file
  src/bag_to_poses_file.cpp
)

target_include_directories(${PROJECT_NAME}_bag_to_poses_file
  PUBLIC
)

target_link_libraries(${PROJECT_NAME}_bag_to_poses_file
  beam::utils
  beam::mapping
  beam::filtering
  beam::calibration
)

add_executable(${PROJECT_NAME}_poses_to_poses_file
  src/poses_to_poses_file.cpp
)

target_include_directories(${PROJECT_NAME}_poses_to_poses_file
  PUBLIC
)

target_link_libraries(${PROJECT_NAME}_poses_to_poses_file
  beam::utils
  beam::mapping
  beam::filtering
  beam::calibration
)

add_executable(${PROJECT_NAME}_append_trajectory
  src/append_trajectory.cpp
)

target_include_directories(${PROJECT_NAME}_append_trajectory
  PUBLIC
)

target_link_libraries(${PROJECT_NAME}_append_trajectory
  beam::utils
  beam::mapping
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################ tests ##################

add_executable(${PROJECT_NAME}_map_builder_tests
  tests/MapBuilderTest.cpp
)

target_include_directories(${PROJECT_NAME}_map_builder_tests
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}_map_builder_tests
  ${PROJECT_NAME}
  Catch2::Catch2
)
