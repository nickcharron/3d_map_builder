#pragma once

#include <octomap/OcTree.h>
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
 * @brief create a custom wrapper around OcTree so I can expose some functions
 * as public
 */
class OcTreeWrapper : public octomap::OcTree {
public:
  OcTreeWrapper(double resolution) : octomap::OcTree(resolution) {}

  bool insertMissedRay(const octomap::point3d& start,
                       const octomap::point3d& end) {
    return integrateMissOnRay(start, end, false);
  }
};

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
  /**
   * @brief main constructor
   * @param bag_file full path to bag file containing the 3D data
   * @param config_file full path to configuration file
   * @param pose_file full path to pose file. For format, see
   * libbeam/beam_mapping/tests/test_data/PosesTests. You can create this using
   * the bag_to_poses executable
   * @param output_directory full path to output directory to save resutls. This
   * must exist.
   * @param extrinsics json file containing extrinsics. For format, see
   * test_data/MapBuilderTests/extrinsics.json
   * @param poses_moving_frame optional moving frame associated with the poses.
   * This needs to match a frame in the extrinsics. If not provided, it will use
   * the frame from the poses file. Otherwise, it will override.
   * @param poses_format_type int specifying i/o format type of pose file.
   */
  MapBuilder(const std::string& bag_file, const std::string& config_file,
             const std::string& pose_file, const std::string& output_directory,
             const std::string& extrinsics,
             const std::string& poses_moving_frame = "",
             int poses_format_type = beam_mapping::format_type::Type1);

  /**
   * @brief delete default constructor
   */
  MapBuilder() = delete;

  /**
   * @brief Default destructor
   */
  ~MapBuilder() = default;

  /**
   * @brief performs the map building
   * @param save_output set to true to generate poses and save map to disk
   */
  void BuildMap(bool save_output = true);

  /**
   * @brief get sensor data for post-processing
   */
  beam_mapping::sensor_data_type GetSensorData() { return sensor_data_; };

private:
  /**
   * @brief method to load poses from json and extrinsics
   */
  void LoadTrajectory();

  /**
   * @brief method for cropping the input point cloud
   * @param cloud point cloud to crop
   * @param sensor_number used for getting crop box parameters
   * @return cropped_cloud
   */
  PointCloud CropCloudRaw(const PointCloud& cloud, uint8_t sensor_number);

  /**
   * @brief method to load configuration from json
   */
  void LoadConfigFromJSON();

  /**
   * @brief processes a point cloud message by first checking if the pose has
   * changed more than the threshold, if so convert it and add to the scans and
   * timestamp vectors
   * @param iter rosbag iterator
   * @param sensor_number
   */
  void ProcessPointCloudMsg(rosbag::View::iterator& iter,
                            uint8_t sensor_number);

  /**
   * @brief loads all the scans from a specific sensor and builds the scans and
   * timestamps vector.
   * @param sensor_number
   */
  void LoadScans(uint8_t sensor_number);

  /**
   * @brief takes all interpolated poses and generates the trajectory for each
   * pose in both the fixed_frame and sensor_frame for each sensor
   */
  void GeneratePoses();

  /**
   * @brief creates an aggregate map for one sensor scan topic
   * @param sensor_number
   */
  void GenerateMap(uint8_t sensor_number);

  /**
   * @brief outputs maps to save directory
   */
  void SaveMaps();

  PointCloud DeskewPointCloud(const pcl::PointCloud<PointXYZIRT>& skewed_cloud,
                              const ros::Time& scan_time,
                              uint8_t sensor_number) const;

  PointCloud
      DeskewPointCloud(const pcl::PointCloud<PointXYZITRRNR>& skewed_cloud,
                       const ros::Time& scan_time, uint8_t sensor_number) const;

  enum class TimeUnit {
    SECONDS = 0,
    NANOSECONDS,
  };

  struct Angles {
    double pitches_sum_rad{0};
    std::set<double> yaws_rad;
  };

  /**
   * @brief calculate & store all scan measurements that did not return a value
   * by checking angle increments compared to median
   * NOTE: this assumed that the pointT type has a field called .ring, and that
   * the lidar rotates about the Z axis
   */
  template <typename pointT>
  void CalculateNonReturns(const pcl::PointCloud<pointT>& cloud,
                           TimeUnit time_unit) {
    scan_non_returns_.push_back(pcl::PointCloud<pcl::PointXY>());

    // Break up cloud into scan lines
    std::map<uint8_t, pcl::PointCloud<pointT>> scan_lines;
    std::map<uint8_t, Angles> scan_angles;
    for (const auto& p : cloud) {
      uint8_t ring = static_cast<uint8_t>(p.ring);
      if (scan_lines.find(ring) == scan_lines.end()) {
        scan_lines.emplace(ring, pcl::PointCloud<pointT>());
        scan_angles.emplace(ring, Angles());
      }
      scan_lines.at(ring).push_back(p);
      double yaw = atan2(p.y, p.x);
      double rxy = std::sqrt(p.y * p.y + p.x * p.x);
      double pitch = atan(p.z / rxy);
      scan_angles.at(ring).pitches_sum_rad += pitch;
      scan_angles.at(ring).yaws_rad.emplace(yaw);
    }

    // iterate through each ring and get average pitch and median change in yaw
    // per measurement
    for (const auto& [ring, angles] : scan_angles) {
      double mean_pitch = angles.pitches_sum_rad / angles.yaws_rad.size();

      // calculate change in yaw in between each measurement and sort in
      // increasing order to be able to calculate the median
      double yaw_last = *angles.yaws_rad.begin();
      std::set<double> change_in_yaw;
      for (auto iter = std::next(angles.yaws_rad.begin());
           iter != angles.yaws_rad.end(); iter++) {
        change_in_yaw.emplace(*iter - yaw_last);
        yaw_last = *iter;
      }
      auto iter_med = change_in_yaw.begin();
      std::advance(iter_med, change_in_yaw.size() / 2);
      double median_delta_yaw = *iter_med;

      // iterate through yaws and if gap between last and next are both
      // greater than 1.5 x median, then consider that a non-return
      for (auto iter = std::next(angles.yaws_rad.begin());
           iter != angles.yaws_rad.end(); iter++) {
        double yaw = *iter;
        double change_in_yaw = yaw - yaw_last;
        if (change_in_yaw < 1.5 * median_delta_yaw) {
          yaw_last = yaw;
          continue;
        }

        // add missing points
        for (double yaw_int = yaw_last + median_delta_yaw;
             yaw_int < yaw - 1.5 * median_delta_yaw;
             yaw_int += median_delta_yaw) {
          pcl::PointXY p;
          p.x = yaw_int;
          p.y = mean_pitch;
          scan_non_returns_.back().push_back(p);
        }
        yaw_last = yaw;
      }
    }
  }

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
  bool octomap_run_filter_;
  double octomap_resolution_;
  double octomap_probability_threshold_;
  bool octomap_estimate_lidar_non_returns_;
  double octomap_non_return_raytrace_d_min_;
  double octomap_non_return_raytrace_d_max_;
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
  std::vector<pcl::PointCloud<pcl::PointXY>> scan_non_returns_; // [yaw, pitch]
  std::vector<PointCloud::Ptr> maps_;
  Eigen::Matrix4d scan_pose_last_;
  beam_mapping::sensor_data_type sensor_data_;
  std::string dateandtime_{""};
  bool prefix_with_date_{false};
  std::unique_ptr<OcTreeWrapper> octomap_;
};

/** @} group mapping */

} // namespace map_builder
