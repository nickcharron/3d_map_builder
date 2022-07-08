#include <gflags/gflags.h>

#include <beam_mapping/MapBuilder.h>
#include <beam_utils/gflags.h>
#include <beam_utils/math.h>

#include <map_builder/PointCloudTF.h>

DEFINE_string(bag_file, "",
              "Full file path to bag file containing the 3D data. ");
DEFINE_validator(bag_file, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(
    config_file, "",
    "Full file path to config file (ex. /path/to/config/config.json)");
DEFINE_validator(config_file, &beam::gflags::ValidateFileMustExist);
DEFINE_string(pose_file, "",
              "full path to pose file. For format, see "
              "libbeam/beam_mapping/tests/test_data/PosesTests. You can create "
              "this using the bag_to_poses executable.");
DEFINE_validator(pose_file, &beam::gflags::ValidateFileMustExist);
DEFINE_string(
    extrinsics, "",
    "Full file path to extrinsics json config file. For format, see "
    "libbeam/beam_mapping/tests/test_data/MapBuilderTests/extrinsics.json");
DEFINE_validator(extrinsics, &beam::gflags::ValidateJsonFileMustExist);
DEFINE_string(poses_moving_frame, "",
              "optional moving frame associated with the poses. This needs to "
              "match a frame in the extrinsics. If not provided, it will use "
              "the frame from the poses file. Otherwise, it will override.");
DEFINE_string(reference_frame, "",
              "sensor frame that will be referenced for hand-eye calibration");
DEFINE_validator(reference_frame, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(candidate_frame, "",
              "sensor frame that will be matched to reference sensor during "
              "hand-eye calibration");
DEFINE_validator(candidate_frame, &beam::gflags::ValidateCannotBeEmpty);

/**
 * @brief Get sensor data for a specified sensor frame
 *
 * @param[in] sensor_data
 * @param[in] sensor_frame
 */
const beam_mapping::scan_data_type GetSensorData(
    beam_mapping::sensor_data_type &sensor_data, const std::string &sensor_frame) {
  if (sensor_data.find(sensor_frame) != sensor_data.end()) {
    return sensor_data[sensor_frame];
  } else {
    BEAM_CRITICAL("Invalid frame: %s. Ensure frame exists.", sensor_frame);
    throw std::invalid_argument{"Invalid frame. Ensure frame exists."};
  }
}

class HandEyeCalibration {
 public:
  /**
   * @brief Explicit Constructor
   *
   * @param[in] scan_data_reference
   * @param[in] scan_data_candidate
   */
  explicit HandEyeCalibration(const beam_mapping::scan_data_type &scan_data_reference,
                              const beam_mapping::scan_data_type &scan_data_candidate)
      : scan_data_reference_(scan_data_reference),
        scan_data_candidate_(scan_data_candidate),
        max_size_(scan_data_reference.first.size()) {
    ros::Time::init();
  };

 private:
  /**
   * @brief Processess scans for publication
   *
   * @param[in] scan_data
   * @param[in] T_SENSOR_ADJUSTEDSENSOR
   * @param[out] reference_cloud_msg
   */
  sensor_msgs::PointCloud2 ProcessScans(
      const beam_mapping::scan_data_type &scan_data,
      const Eigen::Matrix4d &T_SENSOR_ADJUSTEDSENSOR =
          Eigen::Matrix4d::Identity()) {
    // transform scans into world frame over window
    PointCloud::Ptr cloud = std::make_unique<PointCloud>();
    for (size_t i = idx_start_; i < (idx_start_ + window_size_); i++) {
      PointCloud cloud_i;
      const Eigen::Matrix4d T_WORLD_ADJUSTEDSENSOR =
          scan_data.first[i] * T_SENSOR_ADJUSTEDSENSOR;
      pcl::transformPointCloud(*scan_data.second[i], cloud_i,
                               T_WORLD_ADJUSTEDSENSOR);
      *cloud += cloud_i;
    }

    return beam::PCLToROS(*cloud, ros::Time(0), map_frame_, 0);
  }

  /**
   * @brief Callback for adjusting extrinsic caliration of candidate sensor with
   * respect to reference
   *
   * @param[in] point_cloud_tf
   */
  void AdjustedExtrinsicsCallback(
      const map_builder::PointCloudTF &point_cloud_tf) {
    if ((point_cloud_tf.idx_start + point_cloud_tf.window_size) > max_size_) {
      BEAM_WARN("starting index and window size exceeds scan data size of %d",
                max_size_);
    }
    if (point_cloud_tf.save_data) {
      // convert message fields to transformation matrix
      Eigen::Matrix4d T_SENSOR_ADJUSTEDSENSOR;
      Eigen::Quaterniond q;
      Eigen::Vector3d p{point_cloud_tf.x, point_cloud_tf.y, point_cloud_tf.z};
      beam::RPYtoQuaternionDeg(point_cloud_tf.roll, point_cloud_tf.pitch,
                               point_cloud_tf.yaw, q);
      beam::QuaternionAndTranslationToTransformMatrix(q, p,
                                                      T_SENSOR_ADJUSTEDSENSOR);

      //  publish reference cloud
      bool is_window_reset = (idx_start_ != point_cloud_tf.idx_start ||
                              window_size_ != point_cloud_tf.window_size);
      if ((publish_reference_ || is_window_reset) &&
          reference_cloud_pub_.getNumSubscribers() > 0) {
        idx_start_ = point_cloud_tf.idx_start;
        window_size_ = point_cloud_tf.window_size;
        publish_reference_ = false;
        BEAM_INFO("Publishing reference cloud ... \r");
        reference_cloud_pub_.publish(ProcessScans(scan_data_reference_));
        BEAM_INFO("published");
      }

      // publish candidate cloud
      if (candidate_cloud_pub_.getNumSubscribers() > 0) {
        BEAM_INFO("Publishing candidate cloud ... \r");
        candidate_cloud_pub_.publish(
            ProcessScans(scan_data_candidate_, T_SENSOR_ADJUSTEDSENSOR));
        BEAM_INFO("published");
      }
    }
  }

  // variables
  size_t idx_start_{0};
  size_t window_size_{200};
  size_t max_size_;
  bool publish_reference_{true};
  std::string map_frame_{"world"};
  Eigen::Vector3f scan_voxel_size_{0.01, 0.01, 0.01};
  beam_mapping::scan_data_type scan_data_reference_;
  beam_mapping::scan_data_type scan_data_candidate_;

  // node properties
  ros::NodeHandle nh_ = ros::NodeHandle{"hand_eye_calibration"};
  ros::Publisher reference_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("reference_cloud", 1);
  ros::Publisher candidate_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("candidate_cloud", 1);
  ros::Subscriber adjusted_extrinsics_sub_ =
      nh_.subscribe("adjusted_extrinsics", 10,
                    &HandEyeCalibration::AdjustedExtrinsicsCallback, this);
};

int main(int argc, char *argv[]) {
  // build map without saving to disk
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  beam_mapping::MapBuilder map_builder(
      FLAGS_bag_file, FLAGS_config_file, FLAGS_pose_file,
      "dummy_output_directory", FLAGS_extrinsics, FLAGS_poses_moving_frame);
  map_builder.BuildMap(false);

  // get reference and candidate scan data
  beam_mapping::sensor_data_type sensor_data = map_builder.GetSensorData();
  beam_mapping::scan_data_type scan_data_reference =
      GetSensorData(sensor_data, FLAGS_reference_frame);
  beam_mapping::scan_data_type scan_data_candidate =
      GetSensorData(sensor_data, FLAGS_candidate_frame);

  // node handle
  BEAM_INFO("Hand-Eye Calibration of reference and candidate sensors");
  try {
    ros::init(argc, argv, "hand_eye_calibration");
    HandEyeCalibration hand_eye_calibration(scan_data_reference,
                                            scan_data_candidate);
    ros::Rate ros_rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      ros_rate.sleep();
    }
  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
