#include <map_builder/MapBuilder.h>

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
                       int poses_format_type, const std::string& lidar_type)
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

  try {
    intermediary_map_size_ = J["intermediary_map_size"];
    min_translation_m_ = J["min_translation_m"];
    min_rotation_deg_ = J["min_rotation_deg"];
    combine_sensor_data_ = J["combine_sensor_data"];
    for (const auto& sensor : J["3d_sensor_data"]) {
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
    output_filters_ =
        beam_filtering::LoadFilterParamsVector(J["output_filters"]);
  } catch (const nlohmann::json::exception& e) {
    BEAM_CRITICAL("Unable to load json, one or more missing or invalid params. "
                  "Reason: {}",
                  e.what());
    throw std::runtime_error{"Invalid json"};
  }
}

void MapBuilder::LoadTrajectory() {
  if (!slam_poses_.LoadFromFile(pose_file_path_, poses_format_type_)) {
    BEAM_CRITICAL(
        "Invalid pose file type. Valid extensions: .ply, .json, .txt, .pcd");
    throw std::invalid_argument{
        "Invalid pose file type. Valid extensions: .ply, .json, .txt, .pcd"};
  }

  if (poses_moving_frame_.empty()) {
    poses_moving_frame_ = slam_poses_.GetMovingFrame();
  }

  if (poses_moving_frame_.empty()) {
    BEAM_CRITICAL("You must provide the moving frame name of the poses either "
                  "by adding it in the poses file, or adding it in the "
                  "MapBuilder constructor. Exiting.");
    throw std::runtime_error{
        "Set moving frame in map builder before building map."};
  }

  if (map_frame_.empty()) { map_frame_ = slam_poses_.GetFixedFrame(); }

  if (map_frame_.empty()) {
    BEAM_WARN("No fixed frame set in poses file, using 'map'");
    map_frame_ = "map";
  }

  extrinsics_.LoadJSON(extrinsics_file_);

  for (size_t k = 0; k < slam_poses_.GetTimeStamps().size(); k++) {
    Eigen::Affine3d T_MAP_MOVINGFRAME(slam_poses_.GetPoses()[k]);
    trajectory_.AddTransform(T_MAP_MOVINGFRAME, map_frame_, poses_moving_frame_,
                             slam_poses_.GetTimeStamps()[k]);
  }
}

PointCloud MapBuilder::CropCloudRaw(const PointCloud& cloud,
                                    uint8_t sensor_number) {
  if (!sensors_[sensor_number].use_cropbox) { return cloud; }
  beam_filtering::CropBox<pcl::PointXYZ> cropper;
  cropper.SetMinVector(sensors_[sensor_number].cropbox_min);
  cropper.SetMaxVector(sensors_[sensor_number].cropbox_max);
  cropper.SetRemoveOutsidePoints(sensors_[sensor_number].remove_outside_points);
  cropper.SetInputCloud(std::make_shared<PointCloud>(cloud));
  cropper.Filter();
  return cropper.GetFilteredCloud();
}

void MapBuilder::ProcessPointCloudMsg(rosbag::View::iterator& iter,
                                      uint8_t sensor_number) {
  auto sensor_msg = iter->instantiate<sensor_msgs::PointCloud2>();
  ros::Time scan_time = sensor_msg->header.stamp;

  if ((scan_time < slam_poses_.GetTimeStamps()[0]) ||
      (scan_time > slam_poses_.GetTimeStamps().back())) {
    return;
  }
  Eigen::Affine3d scan_pose_current =
      trajectory_.GetTransformEigen(map_frame_, poses_moving_frame_, scan_time);

  if (!beam::PassedMotionThreshold(scan_pose_last_, scan_pose_current.matrix(),
                                   min_rotation_deg_, min_translation_m_, true,
                                   false, false)) {
    return;
  }

  pcl::PCLPointCloud2::Ptr pcl_pc2_tmp =
      std::make_shared<pcl::PCLPointCloud2>();
  PointCloud cloud_tmp;
  beam::pcl_conversions::toPCL(*sensor_msg, *pcl_pc2_tmp);
  pcl::fromPCLPointCloud2(*pcl_pc2_tmp, cloud_tmp);
  PointCloud cloud_cropped = CropCloudRaw(cloud_tmp, sensor_number);
  PointCloud cloud_filtered = beam_filtering::FilterPointCloud<pcl::PointXYZ>(
      cloud_cropped, input_filters_);
  scans_.push_back(std::make_shared<PointCloud>(cloud_filtered));
  interpolated_poses_.AddSinglePose(scan_pose_current.matrix());
  interpolated_poses_.AddSingleTimeStamp(scan_time);
  scan_pose_last_ = scan_pose_current.matrix();
}

void MapBuilder::LoadScans(uint8_t sensor_number) {
  // load all params specific to this sensor
  std::string scan_frame = sensors_[sensor_number].frame;
  std::string scan_topic = sensors_[sensor_number].topic;

  // load rosbag and create view
  rosbag::Bag bag;
  try {
    bag.open(bag_file_path_, rosbag::bagmode::Read);
  } catch (rosbag::BagException& ex) {
    BEAM_CRITICAL("Bag exception : {}}", ex.what());
  }
  rosbag::View view(bag, rosbag::TopicQuery(scan_topic), ros::TIME_MIN,
                    ros::TIME_MAX, true);
  int total_messages = view.size();
  int message_counter = 0;
  std::string output_message = "Loading scans for sensor No. ";
  for (auto iter = view.begin(); iter != view.end(); iter++) {
    message_counter++;
    output_message += std::to_string(sensor_number + 1);
    beam::OutputPercentComplete(message_counter, total_messages,
                                output_message);
    ProcessPointCloudMsg(iter, sensor_number);
  }
}

void MapBuilder::GenerateMap(uint8_t sensor_number) {
  std::string moving_frame = poses_moving_frame_;
  std::string sensor_frame = sensors_[sensor_number].frame;
  PointCloud::Ptr scan_aggregate = std::make_shared<PointCloud>();
  PointCloud::Ptr scan_intermediary = std::make_shared<PointCloud>();
  Eigen::Matrix4d T_MOVING_LIDAR =
      extrinsics_.GetTransformEigen(moving_frame, sensor_frame).matrix();

  // iterate through all scans
  int intermediary_size = 0;
  Eigen::Matrix4d T_FIXED_INT = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_INT_LIDAR = Eigen::Matrix4d::Identity();

  const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses =
      interpolated_poses_.GetPoses();
  std::vector<Eigen::Matrix4d> scan_poses;
  for (uint32_t k = 0; k < poses.size(); k++) {
    intermediary_size++;

    // get the transforms we will need:
    Eigen::Matrix4d T_FIXED_MOVING = poses[k];
    Eigen::Matrix4d T_FIXED_LIDAR = T_FIXED_MOVING * T_MOVING_LIDAR;
    scan_poses.push_back(T_FIXED_LIDAR);
    if (intermediary_size == 1) { T_FIXED_INT = T_FIXED_LIDAR; }
    T_INT_LIDAR = beam::InvertTransform(T_FIXED_INT) * T_FIXED_LIDAR;

    PointCloud::Ptr scan_intermediate_frame = std::make_shared<PointCloud>();
    PointCloud::Ptr intermediary_transformed = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*scans_[k], *scan_intermediate_frame, T_INT_LIDAR);
    *scan_intermediary += *scan_intermediate_frame;

    if (intermediary_size == intermediary_map_size_ || k == scans_.size()) {
      *scan_intermediate_frame =
          beam_filtering::FilterPointCloud<pcl::PointXYZ>(
              *scan_intermediary, intermediary_filters_);
      pcl::transformPointCloud(*scan_intermediate_frame,
                               *intermediary_transformed, T_FIXED_INT);
      *scan_aggregate += *intermediary_transformed;
      scan_intermediary->clear();
      intermediary_size = 0;
    }
  }
  PointCloud new_map = beam_filtering::FilterPointCloud<pcl::PointXYZ>(
      *scan_aggregate, output_filters_);
  maps_.push_back(std::make_shared<PointCloud>(new_map));
  sensor_data_[sensor_frame] = std::make_pair(scan_poses, scans_);
}

void MapBuilder::SaveMaps() {
  for (uint8_t i = 0; i < maps_.size(); i++) {
    std::string save_path = beam::CombinePaths(save_dir_, dateandtime_);
    if (maps_.size() > 1) {
      save_path =
          beam::CombinePaths(save_path, "map_" + sensors_[i].frame + ".pcd");
    } else {
      save_path = beam::CombinePaths(save_path, "map.pcd");
    }

    BEAM_INFO("Saving map to: {}", save_path);
    std::string error_message{};
    if (!beam::SavePointCloud<pcl::PointXYZ>(
            save_path, *maps_[i], beam::PointCloudFileType::PCDBINARY,
            error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
  }
  if (combine_sensor_data_ && maps_.size() > 1) {
    PointCloud::Ptr combined_map = std::make_shared<PointCloud>();
    for (uint8_t i = 0; i < maps_.size(); i++) { *combined_map += *maps_[i]; }
    std::string save_path = save_dir_ + dateandtime_ + "/combined.pcd";
    BEAM_INFO("Saving map to: {}", save_path);
    std::string error_message;
    if (!beam::SavePointCloud<pcl::PointXYZ>(
            save_path, *combined_map, beam::PointCloudFileType::PCDBINARY,
            error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
  }
}

void MapBuilder::GeneratePoses() {
  const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses =
      interpolated_poses_.GetPoses();
  PointCloudCol frame = beam::CreateFrameCol();

  // generate trajectory of moving frame
  PointCloudCol fixed_frame_traj;
  for (size_t k = 0; k < poses.size(); k++) {
    const Eigen::Matrix4d& T_MAP_FIXEDFRAME = poses.at(k);
    beam::MergeFrameToCloud(fixed_frame_traj, frame, T_MAP_FIXEDFRAME);
  }

  // save
  std::string save_path = save_dir_ + dateandtime_ + "/moving_frame_poses.pcd";
  BEAM_INFO("Saving trajetory cloud of moving frame to: {}", save_path);
  std::string error_message;
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(
          save_path, fixed_frame_traj, beam::PointCloudFileType::PCDBINARY,
          error_message)) {
    BEAM_ERROR("Unable to save trajetory cloud. Reason: {}", error_message);
  }

  // generate trajectory of each sensor frame
  for (size_t i = 0; i < sensors_.size(); i++) {
    std::string scan_frame = sensors_[i].frame;
    Eigen::Matrix4d T_MOVING_LIDAR =
        extrinsics_.GetTransformEigen(poses_moving_frame_, scan_frame).matrix();
    PointCloudCol sensor_frame_traj;
    for (size_t k = 0; k < poses.size(); k++) {
      const Eigen::Matrix4d& T_FIXED_MOVING = poses.at(k);
      Eigen::Matrix4d T_FIXED_LIDAR = T_FIXED_MOVING * T_MOVING_LIDAR;
      beam::MergeFrameToCloud(sensor_frame_traj, frame, T_FIXED_LIDAR);
    }

    // save
    save_path = save_dir_ + dateandtime_ + "/sensor_frame_" +
                std::to_string(i) + "_poses.pcd";
    BEAM_INFO("Saving trajetory cloud of sensor frame to: {}", save_path);
    std::string error_message;
    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            save_path, sensor_frame_traj, beam::PointCloudFileType::PCDBINARY,
            error_message)) {
      BEAM_ERROR("Unable to save trajetory cloud. Reason: {}", error_message);
    }
  }
}

void MapBuilder::BuildMap(bool save_output) {
  if (prefix_with_date_) {
    dateandtime_ = beam::ConvertTimeToDate(std::chrono::system_clock::now());
    boost::filesystem::create_directory(save_dir_ + dateandtime_ + "/");
  }

  LoadTrajectory();
  for (uint8_t i = 0; i < sensors_.size(); i++) {
    scan_pose_last_ = Eigen::Matrix4d::Identity();
    interpolated_poses_.Clear();
    scans_.clear();
    LoadScans(i);
    GenerateMap(i);
  }
  if (save_output) {
    GeneratePoses();
    SaveMaps();
  };
}

} // namespace map_builder
