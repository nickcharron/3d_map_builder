#include <gflags/gflags.h>

#include <beam_mapping/MapBuilder.h>
#include <beam_utils/gflags.h>

#include <iostream>

DEFINE_string(
    config_file, "",
    "Full file path to config file (ex. /path/to/config/file.json (or .ply))");
DEFINE_validator(config_file, &beam::gflags::ValidateFileMustExist);
DEFINE_string(trajectory_topic, "", "Optional topic containing the trajectory. This will create a pose file that overrides the one in the config. This topic can either be of type nav_msgs/Odometry, or nav_msgs/Path. If it is of type Path, then this will use the final path message as the trajectory.");

int main(int argc, char *argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  beam_mapping::MapBuilder map_builder(FLAGS_config_file);
  if (!FLAGS_trajectory_topic.empty()){
    std::string bag_file = map_builder.GetBagFile();
    std::string save_dir = map_builder.GetSaveDir();
    beam_mapping::Poses bag_poses;
    BEAM_INFO("Loading poses from bag: {} and topic: {}", bag_file,
              FLAGS_trajectory_topic);
    bag_poses.LoadFromBAG(bag_file, FLAGS_trajectory_topic);
    std::string filepath = save_dir;
    char last_char = filepath.back();
    if (last_char == '/') {
      filepath += "poses.json";
    }
    else {
      filepath += "/poses.json";
    }
    bag_poses.WriteToJSON(filepath);
    map_builder.OverridePoseFile(filepath);
  }
  map_builder.BuildMap();
  return 0;
}
