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
  filtering
  )

find_package(
  catkin REQUIRED
  COMPONENTS
    rosbag
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    )
    
find_package(gflags REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS 
    rosbag
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
  DEPENDS
    Boost
    EIGEN3
)

################ Executables ###################
add_executable(${PROJECT_NAME}_build_map
  src/build_map.cpp
)

target_include_directories(${PROJECT_NAME}_build_map
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_build_map
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  beam::utils
  beam::mapping
  beam::filtering
  beam::calibration
  gflags
)

add_executable(${PROJECT_NAME}_loop_closed_paths_to_poses
  src/loop_closed_paths_to_poses.cpp
)

target_include_directories(${PROJECT_NAME}_loop_closed_paths_to_poses
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_loop_closed_paths_to_poses
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  beam::utils
  beam::mapping
  beam::filtering
  beam::calibration
  gflags
)

add_executable(${PROJECT_NAME}_path_messages_to_poses_files
  src/path_messages_to_pose_files.cpp
)

target_include_directories(${PROJECT_NAME}_path_messages_to_poses_files
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_path_messages_to_poses_files
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  beam::utils
  beam::mapping
  beam::filtering
  beam::calibration
  gflags
)

add_executable(${PROJECT_NAME}_bag_to_poses_file
  src/bag_to_poses_file.cpp
)

target_include_directories(${PROJECT_NAME}_bag_to_poses_file
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_bag_to_poses_file
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  beam::utils
  beam::mapping
  beam::filtering
  beam::calibration
  gflags
)
