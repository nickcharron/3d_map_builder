#include <map_builder/MapBuilderTest.h>

#include <fstream>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_utils/angles.h>
#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/pcl_conversions.h>
#include <beam_utils/pointclouds.h>
#include <beam_utils/se3.h>

using namespace beam_mapping;

namespace map_builder {

MapBuilder::MapBuilder(const std::string& bag_file,
                       const std::string& config_file,
                       const std::string& pose_file,
                       const std::string& output_directory,
                       const std::string& extrinsics,
                       const std::string& poses_moving_frame,
                       int poses_format_type)
    : bag_file_path_(bag_file),
      config_file_(config_file),
      pose_file_path_(pose_file),
      save_dir_(output_directory),
      extrinsics_file_(extrinsics),
      poses_moving_frame_(poses_moving_frame),
      poses_format_type_(poses_format_type) {
  LoadConfigFromJSON();
}

void MapBuilder::LoadConfigFromJSON() {
  BEAM_INFO("Loading MapBuilder config file: {}", config_file_);
  nlohmann::json J;
  if (!beam::ReadJson(config_file_, J)) {
    throw std::runtime_error{"Invalid json"};
  }

  if (!J.contains("intermediary_map_size") ||
      !J.contains("min_translation_m") || !J.contains("min_rotation_deg") ||
      !J.contains("combine_sensor_data") || !J.contains("3d_sensor_data") ||
      !J.contains("input_filters") || !J.contains("intermediary_filters") ||
      !J.contains("output_filters") || !J.contains("deskew_scans") ||
      !J.contains("lidar_type")) {
    BEAM_CRITICAL(
        "Unable to load json, one or more missing or invalid params.");
    throw std::runtime_error{"Invalid json"};
  }

  intermediary_map_size_ = J["intermediary_map_size"];
  min_translation_m_ = J["min_translation_m"];
  min_rotation_deg_ = J["min_rotation_deg"];
  combine_sensor_data_ = J["combine_sensor_data"];
  deskew_scans_ = J["deskew_scans"];
  lidar_type_ = J["lidar_type"];
  for (const auto& sensor : J["3d_sensor_data"]) {
    if (!sensor.contains("topic") || !sensor.contains("frame") ||
        !sensor.contains("use_cropbox") ||
        !sensor.contains("remove_outside_points") ||
        !sensor.contains("cropbox_min") || !sensor.contains("cropbox_max")) {
      BEAM_CRITICAL(
          "Unable to load json, one or more missing or invalid params.");
      throw std::runtime_error{"Invalid json"};
    }
    MapBuilder::SensorConfig sensor_config;
    sensor_config.topic = sensor["topic"];
    sensor_config.frame = sensor["frame"];
    sensor_config.use_cropbox = sensor["use_cropbox"];
    sensor_config.remove_outside_points = sensor["remove_outside_points"];
    std::vector<float> min_ = sensor["cropbox_min"];
    std::vector<float> max_ = sensor["cropbox_max"];
    Eigen::Vector3f min(min_[0], min_[1], min_[2]);
    Eigen::Vector3f max(max_[0], max_[1], max_[2]);
    sensor_config.cropbox_min = min;
    sensor_config.cropbox_max = max;
    sensors_.push_back(sensor_config);
  }
  input_filters_ = beam_filtering::LoadFilterParamsVector(J["input_filters"]);
  intermediary_filters_ =
      beam_filtering::LoadFilterParamsVector(J["intermediary_filters"]);
  output_filters_ = beam_filtering::LoadFilterParamsVector(J["output_filters"]);
}

void MapBuilder::BuildMap(bool save_output) {
  // Load poses
  nlohmann::json J;
  std::map<ros::Time, Eigen::Matrix4d> Ts_World_Imu;
  beam::ReadJson(pose_file_path_, J);
  for (const auto& p_J : J["poses"]) {
    std::vector<double> p = p_J["transform"];
    Eigen::Matrix4d T_World_Imu;
    T_World_Imu << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9],
        p[10], p[11], p[12], p[13], p[14], p[15];
    int nsecs = p_J["time_stamp_nsec"];
    int secs = p_J["time_stamp_sec"];
    ros::Time t(secs, nsecs);
    Ts_World_Imu.emplace(t, T_World_Imu);
  }

  // load calibration
  Eigen::Matrix4d T_LidarH_F1;

  // from beam_slam calibration
  T_LidarH_F1 << 0.00234802, -0.0410186, 0.99915600, 0.15109400, -0.99985700,
      -0.0168436, 0.00165818, -0.00254265, 0.01676140, -0.9990160, -0.04105220,
      -0.03152150, 0.00000000, 0.0000000, 0.00000000, 1.00000000;

  // From beam_robotics/calibration
  // T_LidarH_F1 << 0.002347319365596401, -0.00612366203806173,
  // 0.9999786529677972,
  //     0.151094, -0.999856916629575, -0.01677588285633999,
  //     0.0022444661822886226, -0.00254265, 0.016761619732176197,
  //     -0.9998406888775436, -0.0061623235490002055, -0.0315215, 0.0, 0.0,
  //     0.0, 1.0;

  // old extrinsics from beam_robotics
  // T_LidarH_F1 << 0.002348019393104, -0.999856761222832, 0.016761379475396,
  //     -0.002368714880683, -0.041018552478323, -0.016843618027141,
  //     -0.999016401709371, -0.0253357, 0.999155626095111, 0.001658182361621,
  //     -0.041052226172803, -0.1522565, 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix4d T_Imu_F1;

  // from beam_slam calibration
  T_Imu_F1 << 0.0100911713149964834, 0.00595533299116755,
      -0.99993134877873082137, -0.1863069653847129277, -0.99994819049714009048,
      0.0013960264693824028, -0.010083026906048929779, -0.002186878973101379161,
      0.0013358828476758878748, 0.99998129238454121897,
      0.0059691119905887303179, 0.014295372680480471319, 0.00000, 0.00000,
      0.00000, 1.00000;

  // From beam_robotics/calibration
  // T_Imu_F1 << -0.05968858007224864, -0.048168707659233645,
  // -0.9970541855944441,
  //     -0.05533282256187527, -0.9982170115892695, 0.0031472231591385125,
  //     0.059606146999927, -0.5717153397757792, 0.00026680095428699424,
  //     0.9988342558146612, -0.04827067669325791, 0.05255917117606345, 0.0,
  //     0.0, 0.0, 1.0;

  // old extrinsics from beam_robotics
  // T_Imu_F1 << 0.0100911713149964834, 0.00595533299116755,
  //     -0.99993134877873082137, -0.1863069653847129277,
  //     -0.99994819049714009048, 0.0013960264693824028,
  //     -0.010083026906048929779, -0.002186878973101379161,
  //     0.0013358828476758878748, 0.99998129238454121897,
  //     0.0059691119905887303179, 0.014295372680480471319, 0.00000, 0.00000,
  //     0.00000, 1.00000;
  Eigen::Matrix4d T_Imu_LidarH = T_Imu_F1 * beam::InvertTransform(T_LidarH_F1);

  // load scan data
  rosbag::Bag bag;
  bag.open(bag_file_path_, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery("/lidar_h/velodyne_points"),
                    ros::TIME_MIN, ros::TIME_MAX, true);

  // iterate through scan data
  PointCloud map;
  for (auto iter = view.begin(); iter != view.end(); iter++) {
    auto sensor_msg = iter->instantiate<sensor_msgs::PointCloud2>();
    ros::Time scan_time = sensor_msg->header.stamp;
    if (scan_time < Ts_World_Imu.begin()->first ||
        scan_time > Ts_World_Imu.rbegin()->first) {
      continue;
    }

    // get pose
    auto lb = Ts_World_Imu.lower_bound(scan_time);
    auto ub = Ts_World_Imu.upper_bound(scan_time);
    Eigen::Matrix4d T_World_Imu;
    if (lb == Ts_World_Imu.end()) {
      T_World_Imu = ub->second;
    } else if (ub == Ts_World_Imu.end()) {
      T_World_Imu = lb->second;
    } else {
      auto dub = std::abs((ub->first - scan_time).toSec());
      auto dlb = std::abs((lb->first - scan_time).toSec());
      if (dub < dlb) {
        T_World_Imu = ub->second;
      } else {
        T_World_Imu = lb->second;
      }
    }

    // convert scan to pcl
    PointCloud scan_in_LidarH;
    pcl::PCLPointCloud2 cloud2;
    beam::pcl_conversions::toPCL(*sensor_msg, cloud2);
    pcl::fromPCLPointCloud2(cloud2, scan_in_LidarH);

    // convert to world and add to map
    auto T_World_LidarH = T_World_Imu * T_Imu_LidarH;
    PointCloud scan_in_World;
    pcl::transformPointCloud(scan_in_LidarH, scan_in_World,
                             Eigen::Affine3d(T_World_LidarH));
    map += scan_in_World;
  }

  std::cout << "Saving map of size: " << map.size() << "\n";
  beam::SavePointCloud("/home/nick/test_map.pcd", map);
}

} // namespace map_builder
