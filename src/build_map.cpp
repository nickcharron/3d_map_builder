#include <gflags/gflags.h>

#include <beam_mapping/MapBuilder.h>
#include <beam_utils/gflags.h>

DEFINE_string(
    bag_file, "",
    "Full file path to bag file containing the 3D data. ");
DEFINE_validator(bag_file, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(
    config_file, "",
    "Full file path to config file (ex. /path/to/config/config.json)");
DEFINE_validator(config_file, &beam::gflags::ValidateFileMustExist);
DEFINE_string(
    pose_file, "",
    "full path to pose file. For format, see libbeam/beam_mapping/tests/test_data/PosesTests. You can create this using the bag_to_poses executable.");
DEFINE_validator(pose_file, &beam::gflags::ValidateFileMustExist);
DEFINE_string(
    output_directory, "",
    "Full file output directory for map to save to. This diectory must exist.");
DEFINE_validator(output_directory, &beam::gflags::ValidateDirMustExist);
DEFINE_string(
    extrinsics, "",
    "Full file path to extrinsics json config file. For format, see libbeam/beam_mapping/tests/test_data/MapBuilderTests/extrinsics.json");
DEFINE_validator(extrinsics, &beam::gflags::ValidateJsonFileMustExist);
DEFINE_string(
    poses_moving_frame, "",
    "optional moving frame associated with the poses. This needs to match a frame in the extrinsics. If not provided, it will use the frame from the poses file. Otherwise, it will override.");

int main(int argc, char *argv[])
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  beam_mapping::MapBuilder map_builder(FLAGS_bag_file, FLAGS_config_file, FLAGS_pose_file,
                                       FLAGS_output_directory, FLAGS_extrinsics,
                                       FLAGS_poses_moving_frame);
  map_builder.BuildMap();
  return 0;
}
