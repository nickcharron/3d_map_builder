cmake_minimum_required(VERSION 3.14)
project(3d_map_builder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(
  beam REQUIRED 
  COMPONENTS 
  utils
  mapping
  calibration
)

################ Executables ###################
add_executable(${PROJECT_NAME}_build_map
  src/build_map.cpp
)

target_include_directories(${PROJECT_NAME}_build_map
  PUBLIC
)

target_link_libraries(${PROJECT_NAME}_build_map
  beam::utils
  beam::mapping
)

add_executable(${PROJECT_NAME}_loop_closed_paths_to_poses
  src/loop_closed_paths_to_poses.cpp
)

target_include_directories(${PROJECT_NAME}_loop_closed_paths_to_poses
  PUBLIC
)

target_link_libraries(${PROJECT_NAME}_loop_closed_paths_to_poses
  beam::utils
  beam::mapping
  beam::filtering
  beam::calibration
)

add_executable(${PROJECT_NAME}_path_messages_to_poses_files
  src/path_messages_to_pose_files.cpp
)

target_include_directories(${PROJECT_NAME}_path_messages_to_poses_files
  PUBLIC
)

target_link_libraries(${PROJECT_NAME}_path_messages_to_poses_files
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
