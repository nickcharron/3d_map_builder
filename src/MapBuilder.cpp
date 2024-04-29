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

  beam::ValidateJsonKeysOrThrow({"intermediary_map_size", "min_translation_m",
                                 "combine_sensor_data", "input_filters",
                                 "output_filters", "lidar_type", "octomap",
                                 "min_rotation_deg", "3d_sensor_data",
                                 "intermediary_filters", "deskew_scans"},
                                J);
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

  // octomap config
  nlohmann::json J_octomap = J["octomap"];
  beam::ValidateJsonKeysOrThrow(
      {"run_octomap_filter", "resolution", "probability_threshold",
       "estimate_lidar_non_returns", "non_return_raytrace_depth"},
      J_octomap);
  octomap_run_filter_ = J_octomap["run_octomap_filter"];
  octomap_resolution_ = J_octomap["resolution"];
  octomap_probability_threshold_ = J_octomap["probability_threshold"];
  if (octomap_probability_threshold_ < 0 ||
      octomap_probability_threshold_ > 1) {
    throw std::invalid_argument{"Invalid probability_threshold argument."};
  }
  if (octomap_resolution_ < 0.001 || octomap_resolution_ > 1) {
    throw std::invalid_argument{"Invalid octomap_resolution_ argument."};
  }
  octomap_estimate_lidar_non_returns_ = J_octomap["estimate_lidar_non_returns"];
  octomap_non_return_raytrace_d_min_ = J_octomap["non_return_raytrace_d_min"];
  octomap_non_return_raytrace_d_max_ = J_octomap["non_return_raytrace_d_max"];
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
    Eigen::Affine3d T_Map_MovingFrame(slam_poses_.GetPoses()[k]);
    trajectory_.AddTransform(T_Map_MovingFrame, map_frame_, poses_moving_frame_,
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

  PointCloud cloud_input;
  if (!deskew_scans_) {
    pcl::PCLPointCloud2 cloud2;
    beam::pcl_conversions::toPCL(*sensor_msg, cloud2);
    pcl::fromPCLPointCloud2(cloud2, cloud_input);
  } else if (lidar_type_ == "VELODYNE") {
    pcl::PointCloud<PointXYZIRT> cloud;
    beam::ROSToPCL(cloud, *sensor_msg);
    cloud_input = DeskewPointCloud(cloud, scan_time, sensor_number);
    if (octomap_run_filter_ && octomap_estimate_lidar_non_returns_) {
      CalculateNonReturns<PointXYZIRT>(cloud, TimeUnit::SECONDS);
    }
  } else if (lidar_type_ == "OUSTER") {
    pcl::PointCloud<PointXYZITRRNR> cloud;
    beam::ROSToPCL(cloud, *sensor_msg);
    cloud_input = DeskewPointCloud(cloud, scan_time, sensor_number);
    if (octomap_run_filter_ && octomap_estimate_lidar_non_returns_) {
      CalculateNonReturns<PointXYZITRRNR>(cloud, TimeUnit::NANOSECONDS);
    }
  } else {
    BEAM_ERROR("Invalid input lidar type: {}, options: VELODYNE, OUSTER",
               lidar_type_);
    throw std::runtime_error{"invalid input lidar type"};
  }

  PointCloud cloud_cropped = CropCloudRaw(cloud_input, sensor_number);
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
  Eigen::Matrix4d T_Moving_Lidar =
      extrinsics_.GetTransformEigen(moving_frame, sensor_frame).matrix();

  // iterate through all scans
  int intermediary_size = 0;
  Eigen::Matrix4d T_Fixed_Int = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_Int_Lidar = Eigen::Matrix4d::Identity();

  const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses =
      interpolated_poses_.GetPoses();
  std::vector<Eigen::Matrix4d> scan_poses;
  for (uint32_t k = 0; k < poses.size(); k++) {
    intermediary_size++;

    // get the transforms we will need:
    Eigen::Matrix4d T_Fixed_Moving = poses[k];
    Eigen::Matrix4d T_Fixed_Lidar = T_Fixed_Moving * T_Moving_Lidar;
    const auto& scan_in_lidar = *scans_.at(k);

    // build octomap
    if (octomap_run_filter_) {
      octomap::Pointcloud octo_pc;
      for (int i = 0; i < scan_in_lidar.size(); i++) {
        Eigen::Vector4d p_in_scan(scan_in_lidar.points.at(i).x,
                                  scan_in_lidar.points.at(i).y,
                                  scan_in_lidar.points.at(i).z, 1);
        Eigen::Vector4d p_in_fixed = T_Fixed_Lidar * p_in_scan;
        octo_pc.push_back(p_in_fixed[0], p_in_fixed[1], p_in_fixed[2]);
      }
      Eigen::Vector3d p_origin = T_Fixed_Lidar.block(0, 3, 3, 1);
      octomap::point3d sensor_origin(p_origin[0], p_origin[1], p_origin[2]);
      octomap_->insertPointCloud(octo_pc, sensor_origin);

      if (octomap_estimate_lidar_non_returns_) {
        // iterate over non-returns and add as pointclouds
        const auto& scan_non_returns = scan_non_returns_.at(k);
        for (int i = 0; i < scan_non_returns.size(); i++) {
          float yaw = scan_non_returns.at(i).x;
          float pitch = scan_non_returns.at(i).y;

          // calculate unit vector
          float z = sin(pitch);
          float dxy = sqrt(1 - z * z);
          float x = dxy * cos(yaw);
          float y = dxy * sin(yaw);

          Eigen::Vector3d dir(x, y, z);
          Eigen::Vector3d start_eig =
              p_origin + octomap_non_return_raytrace_d_min_ * dir;
          Eigen::Vector3d end_eig =
              p_origin + octomap_non_return_raytrace_d_max_ * dir;
          octomap::point3d start(start_eig[0], start_eig[1], start_eig[2]);
          octomap::point3d end(end_eig[0], end_eig[1], end_eig[2]);
          octomap_->insertMissedRay(start, end);
        }
      }
    }

    scan_poses.push_back(T_Fixed_Lidar);
    if (intermediary_size == 1) { T_Fixed_Int = T_Fixed_Lidar; }
    T_Int_Lidar = beam::InvertTransform(T_Fixed_Int) * T_Fixed_Lidar;
    PointCloud::Ptr scan_intermediate_frame = std::make_shared<PointCloud>();
    PointCloud::Ptr intermediary_transformed = std::make_shared<PointCloud>();
    pcl::transformPointCloud(scan_in_lidar, *scan_intermediate_frame,
                             T_Int_Lidar);
    *scan_intermediary += *scan_intermediate_frame;
    if (intermediary_size == intermediary_map_size_ || k == scans_.size()) {
      *scan_intermediate_frame =
          beam_filtering::FilterPointCloud<pcl::PointXYZ>(
              *scan_intermediary, intermediary_filters_);
      pcl::transformPointCloud(*scan_intermediate_frame,
                               *intermediary_transformed, T_Fixed_Int);
      *scan_aggregate += *intermediary_transformed;
      scan_intermediary->clear();
      intermediary_size = 0;
    }
  }

  // build octomap
  PointCloud new_map;
  if (octomap_run_filter_) {
    PointCloud map_filtered;
    BEAM_INFO("Running octomap filter");
    for (int i = 0; i < scan_aggregate->size(); i++) {
      auto p = scan_aggregate->at(i);
      octomap::point3d p_octo(p.x, p.y, p.z);
      auto node = octomap_->search(p_octo);
      if (node != NULL &&
          node->getOccupancy() >= octomap_probability_threshold_) {
        map_filtered.push_back(p);
      }
    }
    BEAM_INFO("Filtered {} points",
              scan_aggregate->size() - map_filtered.size());
    new_map = beam_filtering::FilterPointCloud<pcl::PointXYZ>(map_filtered,
                                                              output_filters_);
  } else {
    new_map = beam_filtering::FilterPointCloud<pcl::PointXYZ>(*scan_aggregate,
                                                              output_filters_);
  }
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
    const Eigen::Matrix4d& T_Map_FixedFrame = poses.at(k);
    beam::MergeFrameToCloud(fixed_frame_traj, frame, T_Map_FixedFrame);
  }

  // save
  std::string save_path =
      beam::CombinePaths(save_dir_ + dateandtime_, "moving_frame_poses.pcd");

  BEAM_INFO("Saving trajectory cloud of moving frame to: {}", save_path);
  std::string error_message;
  if (!beam::SavePointCloud<pcl::PointXYZRGB>(
          save_path, fixed_frame_traj, beam::PointCloudFileType::PCDBINARY,
          error_message)) {
    BEAM_ERROR("Unable to save trajectory cloud. Reason: {}", error_message);
  }

  // generate trajectory of each sensor frame
  for (size_t i = 0; i < sensors_.size(); i++) {
    std::string scan_frame = sensors_[i].frame;
    Eigen::Matrix4d T_Moving_Lidar =
        extrinsics_.GetTransformEigen(poses_moving_frame_, scan_frame).matrix();
    PointCloudCol sensor_frame_traj;
    for (size_t k = 0; k < poses.size(); k++) {
      const Eigen::Matrix4d& T_Fixed_Moving = poses.at(k);
      Eigen::Matrix4d T_Fixed_Lidar = T_Fixed_Moving * T_Moving_Lidar;
      beam::MergeFrameToCloud(sensor_frame_traj, frame, T_Fixed_Lidar);
    }

    // save
    save_path =
        beam::CombinePaths(save_dir_ + dateandtime_,
                           "sensor_frame_" + std::to_string(i) + "_poses.pcd");
    BEAM_INFO("Saving trajectory cloud of sensor frame to: {}", save_path);
    std::string error_message;
    if (!beam::SavePointCloud<pcl::PointXYZRGB>(
            save_path, sensor_frame_traj, beam::PointCloudFileType::PCDBINARY,
            error_message)) {
      BEAM_ERROR("Unable to save trajectory cloud. Reason: {}", error_message);
    }
  }
}

void MapBuilder::BuildMap(bool save_output) {
  if (prefix_with_date_) {
    dateandtime_ = beam::ConvertTimeToDate(std::chrono::system_clock::now());
    boost::filesystem::create_directory(save_dir_ + dateandtime_);
  }

  LoadTrajectory();
  if (octomap_run_filter_) {
    octomap_ = std::make_unique<OcTreeWrapper>(octomap_resolution_);
  }

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

PointCloud MapBuilder::DeskewPointCloud(
    const pcl::PointCloud<PointXYZIRT>& skewed_cloud,
    const ros::Time& scan_time, uint8_t sensor_number) const {
  std::string sensor_frame = sensors_[sensor_number].frame;
  Eigen::Affine3d T_Moving_Lidar =
      extrinsics_.GetTransformEigen(poses_moving_frame_, sensor_frame);
  Eigen::Affine3d T_Map_Moving =
      trajectory_.GetTransformEigen(map_frame_, poses_moving_frame_, scan_time);
  Eigen::Affine3d T_Lidar0_Map = (T_Map_Moving * T_Moving_Lidar).inverse();

  PointCloud cloud_deskewed;
  int skipped = 0;
  for (const auto& p : skewed_cloud) {
    ros::Time pt = scan_time + ros::Duration(p.time);
    if (pt < trajectory_.GetStartTime() || pt > trajectory_.GetEndTime()) {
      skipped++;
      continue;
    }
    Eigen::Affine3d T_Map_MovingN =
        trajectory_.GetTransformEigen(map_frame_, poses_moving_frame_, pt);

    Eigen::Affine3d T_Lidar0_LidarN =
        T_Lidar0_Map * T_Map_MovingN * T_Moving_Lidar;
    PointXYZIRT p_deskewed =
        pcl::transformPoint<PointXYZIRT>(p, T_Lidar0_LidarN.cast<float>());
    pcl::PointXYZ p_xyz;
    p_xyz.x = p_deskewed.x;
    p_xyz.y = p_deskewed.y;
    p_xyz.z = p_deskewed.z;
    cloud_deskewed.push_back(p_xyz);
  }
  if (skipped != 0) {
    BEAM_WARN(
        "Skipped deskewing {} points as they were out of the trajectory range",
        skipped);
  }
  return cloud_deskewed;
}

PointCloud MapBuilder::DeskewPointCloud(
    const pcl::PointCloud<PointXYZITRRNR>& skewed_cloud,
    const ros::Time& scan_time, uint8_t sensor_number) const {
  std::string sensor_frame = sensors_[sensor_number].frame;
  Eigen::Affine3d T_Moving_Lidar =
      extrinsics_.GetTransformEigen(poses_moving_frame_, sensor_frame);
  Eigen::Affine3d T_Map_Moving =
      trajectory_.GetTransformEigen(map_frame_, poses_moving_frame_, scan_time);
  Eigen::Affine3d T_Lidar0_Map = (T_Map_Moving * T_Moving_Lidar).inverse();

  PointCloud cloud_deskewed;
  int skipped = 0;
  for (const auto& p : skewed_cloud) {
    ros::Time pt = scan_time + ros::Duration(p.time);
    if (pt < trajectory_.GetStartTime() || pt > trajectory_.GetEndTime()) {
      skipped++;
      continue;
    }
    Eigen::Affine3d T_Map_MovingN =
        trajectory_.GetTransformEigen(map_frame_, poses_moving_frame_, pt);
    Eigen::Affine3d T_Lidar0_LidarN =
        T_Lidar0_Map * T_Map_MovingN * T_Moving_Lidar;
    PointXYZITRRNR p_deskewed =
        pcl::transformPoint<PointXYZITRRNR>(p, T_Lidar0_LidarN.cast<float>());
    pcl::PointXYZ p_xyz;
    p_xyz.x = p_deskewed.x;
    p_xyz.y = p_deskewed.y;
    p_xyz.z = p_deskewed.z;
    cloud_deskewed.push_back(p_xyz);
  }
  if (skipped != 0) {
    BEAM_WARN(
        "Skipped deskewing {} points as they were out of the trajectory range",
        skipped);
  }
  return cloud_deskewed;
}

} // namespace map_builder
