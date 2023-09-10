#include <gflags/gflags.h>

#include <beam_calibration/TfTree.h>
#include <beam_mapping/Poses.h>
#include <beam_mapping/Utils.h>
#include <beam_utils/gflags.h>
#include <beam_utils/se3.h>

#include <iostream>

DEFINE_string(poses_high_rate, "",
              "Full path to pose file that contains the high-rate poses used "
              "to fill in the low-rate poses");
DEFINE_validator(poses_high_rate, &beam::gflags::ValidateFileMustExist);
DEFINE_string(poses_low_rate, "",
              "Full path to pose file that contains the low-rate poses to be "
              "fill in with the high-rate poses");
DEFINE_validator(poses_low_rate, &beam::gflags::ValidateFileMustExist);
DEFINE_string(output_path, "",
              "Full path to output directory. Directory must exist.");
DEFINE_validator(output_path, &beam::gflags::ValidateDirMustExist);
DEFINE_string(
    output_type, "JSON",
    "Type of pose file to output. Default: JSON. Options: JSON, PLY, TXT, PCD");

DEFINE_string(extrinsics, "",
              "Full file path to extrinsics json config file. This is only "
              "needed if the frames of your poses files are different.");

using namespace beam_mapping;

pose_map_type ToPoseMapType(const beam_mapping::Poses& poses,
                            const Eigen::Matrix4d& T) {
  pose_map_type pose_map;
  const auto& timestamps = poses.GetTimeStamps();
  const auto& poses_vec = poses.GetPoses();

  if (timestamps.size() != poses_vec.size()) {
    throw std::runtime_error{"Invalid poses"};
  }
  for (int i = 0; i < poses_vec.size(); i++) {
    Eigen::Matrix4d pose = T * poses_vec.at(i);
    pose_map.emplace(timestamps.at(i).toNSec(), pose);
  }
  return pose_map;
}

beam_mapping::Poses
    FillInTrajectory(const beam_mapping::Poses& poses_hr,
                     const beam_mapping::Poses& poses_lr,
                     const Eigen::Matrix4d& T_MovingLr_MovingHr) {
  pose_map_type high_rate_poses = ToPoseMapType(poses_hr, T_MovingLr_MovingHr);
  pose_map_type low_rate_poses =
      ToPoseMapType(poses_lr, Eigen::Matrix4d::Identity());

  // iterate through HR poses and build corrections
  beam_calibration::TfTree corrections;
  Eigen::Matrix4d T_WORLDCORR_WORLDEST;
  beam_calibration::TfTree transforms_HR;
  auto iter_HR_prev = high_rate_poses.begin();
  auto iter_LR = low_rate_poses.begin();
  for (auto iter_HR = high_rate_poses.begin(); iter_HR != high_rate_poses.end();
       iter_HR++) {
    // get time of curent HR and LR poses
    const uint64_t& t_HR = iter_HR->first;
    const uint64_t& t_LR = iter_LR->first;

    // if time of current HR pose is greater or equal to LR pose, then add
    // correction and increment LR iter
    if (t_HR >= t_LR && iter_LR != low_rate_poses.end()) {
      // get stamp of LR pose
      ros::Time stamp_LR;
      stamp_LR.fromNSec(t_LR);

      // if time of current HR pose is greater than LR pose, then interpolate a
      // HR pose at time of LR pose
      Eigen::Matrix4d T_WORLDEST_BASELINKHR;
      if (t_HR > t_LR) {
        // add current and previous transforms to buffer
        const Eigen::Affine3d T_WB_HR_prev(iter_HR_prev->second);
        const Eigen::Affine3d T_WB_HR_curr(iter_HR->second);

        ros::Time stamp_curr;
        stamp_curr.fromNSec(t_HR);

        ros::Time stamp_prev;
        stamp_prev.fromNSec(iter_HR_prev->first);

        transforms_HR.AddTransform(T_WB_HR_prev, "W", "B", stamp_prev);
        transforms_HR.AddTransform(T_WB_HR_curr, "W", "B", stamp_curr);

        // get transform at time of LR pose
        T_WORLDEST_BASELINKHR =
            transforms_HR.GetTransformEigen("W", "B", stamp_LR).matrix();
      } else {
        T_WORLDEST_BASELINKHR = iter_HR->second;
      }

      // get correction at time of LR pose
      const Eigen::Matrix4d& T_WORLD_BASELINKLR = iter_LR->second;
      T_WORLDCORR_WORLDEST =
          T_WORLD_BASELINKLR * beam::InvertTransform(T_WORLDEST_BASELINKHR);

      // add correction to tf tree
      Eigen::Affine3d T(T_WORLDCORR_WORLDEST);
      corrections.AddTransform(T, "WORLD_CORRECTED", "WORLD_ESTIMATED",
                               stamp_LR);

      // increment LR iterator
      iter_LR++;
    }
    // reset previous HR iterator
    iter_HR_prev = iter_HR;
  }

  // if last HR pose is after last LR pose, then add a correction equal to the
  // final correction
  if (low_rate_poses.rbegin()->first < high_rate_poses.rbegin()->first) {
    Eigen::Affine3d T(T_WORLDCORR_WORLDEST);
    ros::Time stamp;
    stamp.fromNSec(high_rate_poses.rbegin()->first);
    corrections.AddTransform(T, "WORLD_CORRECTED", "WORLD_ESTIMATED", stamp);
  }

  // Correct all HR poses by interpolating corrections
  beam_mapping::Poses final_poses;
  for (auto iter_HR = high_rate_poses.begin(); iter_HR != high_rate_poses.end();
       iter_HR++) {
    const uint64_t& t_HR = iter_HR->first;
    const Eigen::Matrix4d& T_WORLDEST_BASELINKHR = iter_HR->second;

    // calculate correction
    ros::Time stamp_HR;
    stamp_HR.fromNSec(t_HR);
    T_WORLDCORR_WORLDEST =
        corrections
            .GetTransformEigen("WORLD_CORRECTED", "WORLD_ESTIMATED", stamp_HR)
            .matrix();

    // correct pose and add
    Eigen::Matrix4d T_WORLDCORR_BASELINKHR =
        T_WORLDCORR_WORLDEST * T_WORLDEST_BASELINKHR;
    final_poses.AddSingleTimeStamp(stamp_HR);
    final_poses.AddSinglePose(T_WORLDCORR_BASELINKHR);
  }

  return final_poses;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  beam_mapping::Poses poses_hr;
  poses_hr.LoadFromFile(FLAGS_poses_high_rate, 1);
  beam_mapping::Poses poses_lr;
  poses_lr.LoadFromFile(FLAGS_poses_low_rate, 1);

  Eigen::Matrix4d T_MovingLr_MovingHr = Eigen::Matrix4d::Identity();
  if (poses_hr.GetMovingFrame() != poses_lr.GetMovingFrame()) {
    if (FLAGS_extrinsics.empty()) {
      BEAM_ERROR(
          "High rate poses file has a moving frame ({}) that is "
          "different from that of the low rate poses ({}), you must "
          "provide an extrinsics path, or check the frames in your poses files",
          poses_hr.GetMovingFrame(), poses_lr.GetMovingFrame());
      throw std::invalid_argument{"Must provide extrinsics path"};
    }

    beam_calibration::TfTree extrinsics;
    extrinsics.LoadJSON(FLAGS_extrinsics);
    T_MovingLr_MovingHr = extrinsics
                              .GetTransformEigen(poses_lr.GetMovingFrame(),
                                                 poses_hr.GetMovingFrame())
                              .matrix();
  }
  beam_mapping::Poses final_poses =
      FillInTrajectory(poses_hr, poses_lr, T_MovingLr_MovingHr);
  final_poses.WriteToFile(FLAGS_output_path, FLAGS_output_type, 1);
  return 0;
}
