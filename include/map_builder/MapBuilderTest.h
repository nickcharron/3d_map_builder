#pragma once

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <beam_calibration/TfTree.h>
#include <beam_filtering/Utils.h>
#include <beam_mapping/Poses.h>
#include <beam_mapping/Utils.h>
#include <beam_utils/math.h>

namespace map_builder {

/**
 * @brief class for map builder
 */
class MapBuilder {
  struct SensorConfig {
    std::string topic;
    std::string frame;
    bool use_cropbox;
    bool remove_outside_points;
    Eigen::Vector3f cropbox_min;
    Eigen::Vector3f cropbox_max;
  };

public:
  MapBuilder(const std::string& bag_file, const std::string& config_file,
             const std::string& pose_file, const std::string& output_directory,
             const std::string& extrinsics,
             const std::string& poses_moving_frame = "",
             int poses_format_type = beam_mapping::format_type::Type1);

  MapBuilder() = delete;
  ~MapBuilder() = default;
  void BuildMap(bool save_output = true);

private:
  void LoadConfigFromJSON();

  // from constructor
  std::string bag_file_path_;
  std::string config_file_;
  std::string pose_file_path_;
  std::string save_dir_;
  std::string extrinsics_file_;
  std::string poses_moving_frame_;
  int poses_format_type_;

  // From Config file
  int intermediary_map_size_;
  double min_translation_m_;
  double min_rotation_deg_;
  bool combine_sensor_data_;
  bool deskew_scans_;
  std::string lidar_type_;
  std::vector<SensorConfig> sensors_;
  std::vector<beam_filtering::FilterParamsType> input_filters_;
  std::vector<beam_filtering::FilterParamsType> intermediary_filters_;
  std::vector<beam_filtering::FilterParamsType> output_filters_;

  // Map Builder objects
  std::string map_frame_;
  beam_mapping::Poses slam_poses_;
  beam_mapping::Poses interpolated_poses_;
  beam_calibration::TfTree trajectory_;
  beam_calibration::TfTree extrinsics_;
  PointCloud::Ptr aggregate_;
  std::vector<PointCloud::Ptr> scans_;
  std::vector<PointCloud::Ptr> maps_;
  Eigen::Matrix4d scan_pose_last_;
  beam_mapping::sensor_data_type sensor_data_;
  std::string dateandtime_{""};
  bool prefix_with_date_{false};
};

/** @} group mapping */

} // namespace map_builder
