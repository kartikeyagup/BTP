#ifndef HELPERS_H
#define HELPERS_H

#include "common.h"
#include <GL/glut.h>

void simulate_images(grid_params grid_description,
  motion_type motion,
  float angle,
  int num_images,
  float distance,
  camera_params intrinsics,
  cv::Point3d starting_point,
  std::vector<camera_frame> &output_frames);

#endif