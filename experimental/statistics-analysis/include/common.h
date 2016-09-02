#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <string>

struct grid_params {
  int gridx;
  int gridy;

  grid_params(int x, int y) {
    gridx = x;
    gridy = y;
  }
};

enum motion_type {
  FORWARD,
  LEFT
};

struct camera_params {
  float fx;
  float fy;
  float cx;
  float cy;

  camera_params(float focalx, float focaly,
    float centerx, float centery) {
    fx = focalx;
    fy = focaly;
    cx = centerx;
    cy = centery;
  }
};

struct camera_frame {
  cv::Mat image;
  Eigen::Matrix3d rotation;
  cv::Point3f position;
  camera_params intrinsics;
};

#endif
