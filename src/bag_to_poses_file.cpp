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
    "type if provided, this will only output the last path message.");
DEFINE_validator(topic, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(output_path, "",
              "Full path to output directory. Directory must exist.");
DEFINE_validator(output_path, &beam::gflags::ValidateDirMustExist);
DEFINE_string(output_type, "JSON",
              "Type of path file to output. Default: JSON. Options: JSON, PLY, "
              "PLY2, TXT");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  beam_mapping::Poses pose_builder;
  pose_builder.LoadFromBAG(FLAGS_bag, FLAGS_topic);
  if (FLAGS_output_type == "JSON") {
    pose_builder.WriteToJSON(FLAGS_output_path);
  } else if (FLAGS_output_type == "PLY") {
    pose_builder.WriteToPLY(FLAGS_output_path);
  } else if (FLAGS_output_type == "PLY2") {
    pose_builder.WriteToPLY2(FLAGS_output_path);
  } else if (FLAGS_output_type == "TXT") {
    pose_builder.WriteToTXT(FLAGS_output_path);
  } else {
    BEAM_ERROR("Invalid output type, using default: JSON");
    pose_builder.WriteToJSON(FLAGS_output_path);
  }

  return 0;
}
