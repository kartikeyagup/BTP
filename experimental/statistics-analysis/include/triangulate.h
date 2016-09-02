#ifndef TRIANGULATE_H
#define TRIANGULATE_H

#include "common.h"

std::vector<std::vector<cv::Point3f> > detect_triangulate(
  std::vector<camera_frame> camera_frames);

#endif
