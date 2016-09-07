#ifndef STATISTICS_H
#define STATISTICS_H

#include "common.h"

void dump_disk(std::vector<std::vector<cv::Point3f> > inputpoints,
  grid_params grid_description,
  motion_type motion,
  float angle,
  int num_images,
  float distance,
  camera_params intrinsics,
  cv::Point3f starting_point,
  std::vector<camera_frame> &cam_frames,
  std::string dump_directory);

#endif
