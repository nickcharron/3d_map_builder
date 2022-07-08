#include <gflags/gflags.h>

#include <beam_mapping/Poses.h>
#include <beam_utils/gflags.h>

#include <iostream>

DEFINE_string(bag, "",
              "Full file path to bag file (ex. /path/to/bag/name.bag)");
DEFINE_validator(bag, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(loop_closed_path_topic, "",
              "Topic associated with the loop closed path");
DEFINE_validator(loop_closed_path_topic, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(high_rate_poses_path_topic, "",
              "Topic associated with the high rate poses path");
DEFINE_validator(high_rate_poses_path_topic,
                 &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(output_path, "",
              "Full path to output directory. Directory must exist.");
DEFINE_validator(output_path, &beam::gflags::ValidateDirMustExist);
DEFINE_string(output_type, "JSON",
              "Type of path file to output. Default: JSON. Options: JSON, PLY, "
              "PLY2, TXT");
DEFINE_int32(format_type, 1,
             "Format type of output. Default: 1. Options: 1, 2, 3");
DEFINE_bool(interpolate_corrections, true,
            "Setting this to true calculates a new correction for each "
            "high-rate pose by interpolating the loop closed poses. This makes "
            "the corrected trajectory continuous.");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // load poses
  beam_mapping::Poses pose_builder;
  if (FLAGS_interpolate_corrections) {
    pose_builder.LoadLoopClosedPathsInterpolated(
        FLAGS_bag, FLAGS_loop_closed_path_topic,
        FLAGS_high_rate_poses_path_topic);
  } else {
    pose_builder.LoadLoopClosedPaths(FLAGS_bag, FLAGS_loop_closed_path_topic,
                                     FLAGS_high_rate_poses_path_topic);
  }

  pose_builder.WriteToFile(FLAGS_output_path, FLAGS_output_type,
                           FLAGS_format_type);

  return 0;
}
