#include <gflags/gflags.h>

#include <beam_calibration/TfTree.h>
#include <beam_mapping/Poses.h>
#include <beam_utils/gflags.h>

#include <iostream>

DEFINE_string(poses, "", "Full path to pose file (ex. /path/to/bag/pose.ext)");
DEFINE_validator(poses, &beam::gflags::ValidateDirMustExist);
DEFINE_string(output_path, "",
              "Full path to output directory. Directory must exist.");
DEFINE_validator(output_path, &beam::gflags::ValidateDirMustExist);
DEFINE_string(
    output_type, "JSON",
    "Type of pose file to output. Default: JSON. Options: JSON, PLY, TXT, PCD");
DEFINE_int32(in_format_type, 1,
             "Format type of output. Default: 1. Options: 1, 2, 3");
DEFINE_int32(out_format_type, 1,
             "Format type of output. Default: 1. Options: 1, 2, 3");
DEFINE_string(desired_poses_moving_frame, "",
              "Optionally specify frame in which poses are too be expressed. "
              "This must match a frame in the extrinsics.");
DEFINE_string(extrinsics, "",
              "Full file path to extrinsics json config file. The "
              "desired_poses_moving_frame id must be specified. For format, "
              "see map_builder/config/examples/EXAMPLE_EXTRINSICS.json");
DEFINE_string(
    poses_moving_frame, "",
    "If desired_poses_moving_frame is specified, optionally specify the "
    "moving frame associated with the poses. This must match a frame in the "
    "extrinsics. If not provided, it will use the frame from the poses file. "
    "Otherwise, it will override.");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  beam_mapping::Poses pose_builder;
  pose_builder.LoadFromFile(FLAGS_poses, FLAGS_in_format_type);

  // optionally transform poses into desired frame
  if (!FLAGS_desired_poses_moving_frame.empty()) {
    // set moving frame
    if (!FLAGS_poses_moving_frame.empty()) {
      pose_builder.SetMovingFrame(FLAGS_poses_moving_frame);
    }
    const std::string poses_moving_frame = pose_builder.GetMovingFrame();
    if (poses_moving_frame.empty()) {
      BEAM_ERROR(
          "pose moving frame is empty and must be set as a command line "
          "argument.");
    }

    // load extrinsics
    beam_calibration::TfTree extrinsics;
    extrinsics.LoadJSON(FLAGS_extrinsics);
    const Eigen::Matrix4d T_MOVINGFRAME_DESIREDMOVINGFRAME =
        extrinsics
            .GetTransformEigen(FLAGS_desired_poses_moving_frame,
                               poses_moving_frame)
            .matrix();

    // transform poses into desired frame
    auto poses = pose_builder.GetPoses();
    for (auto& T_WORLD_MOVINGFRAME : poses) {
      T_WORLD_MOVINGFRAME *= T_MOVINGFRAME_DESIREDMOVINGFRAME;
    }
    pose_builder.SetPoses(poses);
  }

  pose_builder.WriteToFile(FLAGS_output_path, FLAGS_output_type,
                           FLAGS_out_format_type);
  return 0;
}
