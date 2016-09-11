#ifndef TRIANGULATE_H
#define TRIANGULATE_H

#include "common.h"

std::unordered_map<TwoDPoint, cv::Point3f> detect_triangulate(
  grid_params &grid_description,
  std::vector<camera_frame> camera_frames);

void ConvertPoint(triangulation_bundle &bundle);

cv::Point3f Triangulate(std::vector<triangulation_bundle> &input);

std::unordered_map<TwoDPoint, cv::Point2f> detect(
  grid_params &grid_description,
  cv::Mat &image);

#endif
