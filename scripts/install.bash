#!/bin/bash
set -e

# Get important directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SRC_DIR="${SCRIPT_DIR//'/3d_map_builder/scripts'}"

USING_CATKIN=0
NUM_PROCESSORS=1

IGNORE_BEAM_CALIBRATION_=0
IGNORE_BEAM_COLORIZE_=1
IGNORE_BEAM_CONTAINERS_=1
IGNORE_BEAM_CV_=1
IGNORE_BEAM_DEFECTS_=1
IGNORE_BEAM_DEPTH_=1
IGNORE_BEAM_FILTERING_=0
IGNORE_BEAM_MAPPING_=0
IGNORE_BEAM_MATCHING_=1
IGNORE_BEAM_OPTIMIZATION_=1
IGNORE_BEAM_UTILS_=0

# get number of processors for high-load installs
if [ $(nproc) -lt 2 ]; then
  NUM_PROCESSORS=1
else
  NUM_PROCESSORS=$(( $(nproc) / 2 ))
fi

get_inputs()
{
  echo "Are you installing to a catkin workspace? (y/n)"
  while read ans; do
      case "$ans" in
          y) USING_CATKIN=1; break;;
          n) break;;
          *) echo "Invalid input (y/n):";;
      esac
  done

}

install_libbeam()
{
  # Ensure that Beam install scripts are installed
  LIBBEAM_SRC="$SRC_DIR/libbeam"
  if [ -d $LIBBEAM_SRC ]; then
      ## TODO: edit this
      echo "libbeam found. MAKE SURE YOU ARE ON VERSION v1.0"
  else
      echo "Cloning libbeam into: $SRC_DIR"
      cd $SRC_DIR 
      
      # TODO: uncomment this and delete the rest once merged and version created
      # git clone --depth 1 -b v1.0 https://github.com/BEAMRobotics/libbeam.git
      git clone --depth 1 -b update_cmake https://github.com/BEAMRobotics/libbeam.git
  fi

  INSTALL_LIBBEAM_LOCALLY_=$USING_CATKIN
  INSTALL_OPENCV4_=0
  INSTALL_OPENCV4_LOCALLY_=$USING_CATKIN
  INSTALL_CERES_=0

  cd $LIBBEAM_SRC
  bash scripts/install.bash \
  INSTALL_LIBBEAM_LOCALLY=$INSTALL_LIBBEAM_LOCALLY_ \
  INSTALL_OPENCV4=$INSTALL_OPENCV4_ \
  INSTALL_OPENCV4_LOCALLY=$INSTALL_OPENCV4_LOCALLY_ \
  INSTALL_CERES=$INSTALL_CERES_ \
  IGNORE_BEAM_CALIBRATION=$IGNORE_BEAM_CALIBRATION_ \
  IGNORE_BEAM_COLORIZE=$IGNORE_BEAM_COLORIZE_ \
  IGNORE_BEAM_CONTAINERS=$IGNORE_BEAM_CONTAINERS_ \
  IGNORE_BEAM_CV=$IGNORE_BEAM_CV_ \
  IGNORE_BEAM_DEFECTS=$IGNORE_BEAM_DEFECTS_ \
  IGNORE_BEAM_DEPTH=$IGNORE_BEAM_DEPTH_ \
  IGNORE_BEAM_FILTERING=$IGNORE_BEAM_FILTERING_ \
  IGNORE_BEAM_MAPPING=$IGNORE_BEAM_MAPPING_ \
  IGNORE_BEAM_MATCHING=$IGNORE_BEAM_MATCHING_ \
  IGNORE_BEAM_OPTIMIZATION=$IGNORE_BEAM_OPTIMIZATION_ \
  IGNORE_BEAM_UTILS=$IGNORE_BEAM_UTILS_
}

install_routine() 
{
  if(( $USING_CATKIN == 1 ))
  then
      cd $SRC_DIR
      echo "Building 3d_map_builder"
      bash 3d_map_builder/scripts/export_libbeam_cmake_variables.bash
      echo "Install complete. Go to your catkin workspace and run catkin build"
  else 
      cd $SRC_DIR
      cd 3d_map_builder
      mkdir -p build
      cd build
      cmake ..
      make -j$NUM_PROCESSORS
  fi
}

main() 
{
    get_inputs
    install_libbeam
    install_routine
}

main $@
