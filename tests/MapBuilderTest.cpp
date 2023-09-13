#define CATCH_CONFIG_MAIN

#include <filesystem>

#include <catch2/catch.hpp>

#include <beam_utils/math.h>

#include <map_builder/MapBuilder.h>

std::string GetDataPath() {
  std::filesystem::path this_filepath(__FILE__);
  auto data_paths = this_filepath / std::filesystem::path("test_data");
  return data_paths.string();
}

std::string data_path_ = GetDataPath();

TEST_CASE("Testing map building with JSON") {
  std::string config_file_path = data_path_ + "TestConfigJSON.json";
  std::string pose_file_path = data_path_ + "TestPosesJSON.json";
  std::string bag_file_path = data_path_ + "TestBagJSON.bag";
  std::string extrinsics_file_path = data_path_ + "extrinsics.json";
  std::string output_dir_path = data_path_ + "tmp/";
  std::filesystem::create_directory(output_dir_path);
  map_builder::MapBuilder map_builder(bag_file_path, config_file_path,
                                      pose_file_path, output_dir_path,
                                      extrinsics_file_path);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  std::filesystem::remove_all(output_dir_path);
}

TEST_CASE("Testing map building with PLY") {
  std::string config_file_path = data_path_ + "TestConfigPLY.json";
  std::string pose_file_path = data_path_ + "TestPosesPLY.ply";
  std::string bag_file_path = data_path_ + "TestBagPLY.bag";
  std::string extrinsics_file_path = data_path_ + "extrinsics.json";
  std::string output_dir_path = data_path_ + "tmp/";
  std::string moving_frame = "vvlp_link";

  std::filesystem::create_directory(output_dir_path);
  map_builder::MapBuilder map_builder(bag_file_path, config_file_path,
                                      pose_file_path, output_dir_path,
                                      extrinsics_file_path, moving_frame);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  std::filesystem::remove_all(output_dir_path);
}

TEST_CASE("Testing map building with PCD") {
  std::string config_file_path = data_path_ + "TestConfigPCD.json";
  std::string pose_file_path = data_path_ + "TestPosesPCD.pcd";
  std::string bag_file_path = data_path_ + "TestBagPCD.bag";
  std::string extrinsics_file_path = data_path_ + "extrinsics.json";
  std::string output_dir_path = data_path_ + "tmp/";

  std::string moving_frame = "base_link";
  std::filesystem::create_directory(output_dir_path);
  map_builder::MapBuilder map_builder(bag_file_path, config_file_path,
                                      pose_file_path, output_dir_path,
                                      extrinsics_file_path, moving_frame);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  std::filesystem::remove_all(output_dir_path);
}

/**
 * tmp test:
TEST_CASE("Testing map building from Bag odometry") {
  std::string config_file_path = data_path_ + "TestConfigBAG.json";
  std::string bag_file_path = data_path_ + "TestBagBAG.bag";
  std::string extrinsics_file_path = data_path_ + "extrinsics.json";
  std::string output_dir_path = data_path_ + "tmp/";

  beam_mapping::Poses bag_poses;
  std::string bag_poses_file_path =
      "/home/alex/robots/data/ig/ig_scan_2019-02-13-19-44-24-loam.bag";
  if (!std::filesystem::exists(bag_poses_file_path)) {
    BEAM_ERROR("Invalid bag file path, file does not exist. Ensure file is "
               "downloaded locally on your machine. Exiting Test. Input: {}",
               bag_poses_file_path);
    return;
  }

  std::string topic = "/ig/loam/lidar_odom";
  bag_poses.LoadFromBAG(bag_poses_file_path, topic);
  bag_poses.WriteToJSON(pose_file_path);
  map_builder::MapBuilder map_builder(config_file_path);
  map_builder.OverrideBagFile(bag_poses_file_path);
  map_builder.OverridePoseFile(pose_file_path + "_poses.json");
  map_builder.OverrideExtrinsicsFile(extrinsics_file_path);
  map_builder.OverrideOutputDir(output_dir_path);
  std::filesystem::create_directory(output_dir_path);
  REQUIRE_NOTHROW(map_builder.BuildMap());
  std::filesystem::remove_all(output_dir_path);
  std::filesystem::remove_all(pose_file_path + "_poses.json");
}
*/
