#include <gflags/gflags.h>

#include <beam_mapping/Poses.h>
#include <beam_utils/gflags.h>
#include <beam_utils/se3.h>

DEFINE_string(trajectory, "", "Full path to initial trajectory json file");
DEFINE_validator(trajectory, &beam::gflags::ValidateJsonFileMustExist);
DEFINE_string(append, "",
              "Full path to trajectory json file that will be added to the end "
              "of the initial");
DEFINE_validator(append, &beam::gflags::ValidateJsonFileMustExist);
DEFINE_string(output_path, "", "Full path to output trajectory");
DEFINE_string(
    output_type, "JSON",
    "Type of pose file to output. Default: JSON. Options: JSON, PLY, TXT, PCD");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  beam_mapping::Poses poses_trajectory;
  poses_trajectory.LoadFromJSON(FLAGS_trajectory);
  auto ts_trajectory = poses_trajectory.GetTimeStamps();
  auto trajectory = poses_trajectory.GetPoses();

  beam_mapping::Poses poses_append;
  poses_append.LoadFromJSON(FLAGS_append);
  auto ts_append = poses_trajectory.GetTimeStamps();
  auto append = poses_trajectory.GetPoses();

  const ros::Time& t_last = ts_trajectory.back();
  const Eigen::Matrix4d& T_WORLD_MOVING1_last = trajectory.back();

  beam_mapping::Poses poses_final = poses_trajectory;
  Eigen::Matrix4d T_WORLD_MOVING2_start;
  bool start_pose_set = false;
  for (int k = 0; k < ts_append.size(); k++) {
    const auto& t = ts_append.at(k);
    if (t <= t_last) {
      continue;
    } else if (t == t_last) {
      T_WORLD_MOVING2_start = append.at(k);
      start_pose_set = true;
      continue;
    }

    const auto& T_WORLD_MOVING2_k = append.at(k);

    if (!start_pose_set) {
      T_WORLD_MOVING2_start = beam::InterpolateTransform(
          append.at(k - 1), ts_append.at(k - 1).toSec(), append.at(k),
          ts_append.at(k).toSec(), t_last.toSec());
      start_pose_set = true;
    }

    poses_final.AddSingleTimeStamp(t);

    Eigen::Matrix4d T_MOVING2Start_MOVING2K =
        beam::InvertTransform(T_WORLD_MOVING2_start) * T_WORLD_MOVING2_k;
    Eigen::Matrix4d T_WORLD_MOVING1_k =
        T_WORLD_MOVING1_last * T_MOVING2Start_MOVING2K;
    poses_final.AddSinglePose(T_WORLD_MOVING1_k);
  }

  poses_final.WriteToFile(FLAGS_output_path, FLAGS_output_type);

  return 0;
}
