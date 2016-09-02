#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <string>

struct grid_params {
  int gridx;
  int gridy;
};

enum motion_type {
  FORWARD,
  LEFT
};

struct camera_params {
  float focal_length;
  float cx;
  float cy;
};

struct camera_frame {
  cv::Mat image;
  Eigen::Matrix3d rotation;
  cv::Point3f position;
  camera_params intrinsics;
};

#endif
