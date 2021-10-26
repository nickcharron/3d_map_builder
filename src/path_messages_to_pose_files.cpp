#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <beam_mapping/Poses.h>
#include <beam_mapping/Utils.h>
#include <beam_utils/gflags.h>
#include <beam_utils/log.h>

#include <iostream>

DEFINE_string(bag, "",
              "Full file path to bag file (ex. /path/to/bag/name.bag)");
DEFINE_validator(bag, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(topic, "", "Topic associated with the path messages.");
DEFINE_validator(topic, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(output_path, "",
              "Full path to output directory. Directory must exist.");
DEFINE_validator(output_path, &beam::gflags::ValidateDirMustExist);
DEFINE_string(output_type, "JSON",
              "Type of path file to output. Default: JSON. Options: JSON, PLY, "
              "PLY2, TXT");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Open bag and load view
  rosbag::Bag bag;
  BEAM_INFO("Opening bag: {}", FLAGS_bag);
  bag.open(FLAGS_bag, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(FLAGS_topic), ros::TIME_MIN,
                    ros::TIME_MAX, true);

  boost::filesystem::path p(FLAGS_bag);
  std::string bag_name = p.stem().string();

  // iterate through all views and save poses file for each
  int counter = 0;
  for (auto iter = view.begin(); iter != view.end(); iter++) {
    counter++;
    // load path msg and confirm type
    boost::shared_ptr<nav_msgs::Path> path_msg =
        iter->instantiate<nav_msgs::Path>();
    if (path_msg == NULL) {
      BEAM_ERROR(
          "Cannot instantiate path msg, make sure your topic is correct.");
      throw std::runtime_error{"Cannot instantiate path msg."};
    }

    // load data
    std::vector<Eigen::Matrix4d, beam::AlignMat4d> poses;
    std::vector<ros::Time> timestamps;
    std::string moving_frame;
    std::string fixed_frame;
    beam_mapping::utils::PathMsgToPoses(*path_msg, poses, timestamps,
                                        fixed_frame, moving_frame);

    // convert to poses
    beam_mapping::Poses pose_builder;
    pose_builder.SetFixedFrame(fixed_frame);
    pose_builder.SetMovingFrame(moving_frame);
    pose_builder.SetBagName(bag_name);
    pose_builder.SetTimeStamps(timestamps);
    pose_builder.SetPoses(poses);

    // output poses
    std::string output_file =
        FLAGS_output_path + "path" + std::to_string(counter);
    if (FLAGS_output_type == "JSON") {
      pose_builder.WriteToJSON(output_file);
    } else if (FLAGS_output_type == "PLY") {
      pose_builder.WriteToPLY(output_file);
    } else if (FLAGS_output_type == "PLY2") {
      pose_builder.WriteToPLY2(output_file);
    } else if (FLAGS_output_type == "TXT") {
      pose_builder.WriteToTXT(output_file);
    } else {
      BEAM_ERROR("Invalid output type, using default: JSON");
      pose_builder.WriteToJSON(output_file);
    }
  }

  return 0;
}
