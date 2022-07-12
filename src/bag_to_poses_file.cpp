#include <gflags/gflags.h>

#include <beam_mapping/Poses.h>
#include <beam_utils/gflags.h>

#include <iostream>

DEFINE_string(bag, "",
              "Full file path to bag file (ex. /path/to/bag/name.bag)");
DEFINE_validator(bag, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(
    topic, "",
    "Topic associated with the odometry or path messages. If a path message "
    "type is provided, this will only output the last path message.");
DEFINE_validator(topic, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(output_path, "",
              "Full path to output directory. Directory must exist.");
DEFINE_validator(output_path, &beam::gflags::ValidateDirMustExist);
DEFINE_string(
    output_type, "JSON",
    "Type of pose file to output. Default: JSON. Options: JSON, PLY, TXT");
DEFINE_int32(format_type, 1,
             "Format type of output. Default: 1. Options: 1, 2, 3");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  beam_mapping::Poses pose_builder;
  pose_builder.LoadFromBAG(FLAGS_bag, FLAGS_topic);
  pose_builder.WriteToFile(FLAGS_output_path, FLAGS_output_type,
                           FLAGS_format_type);
  return 0;
}
