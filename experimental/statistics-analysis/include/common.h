#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <math.h>

#define PI 3.14159265

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
  RIGHT
};

struct camera_params {
  float fx;
  float fy;
  float cx;
  float cy;

  camera_params() { };

  camera_params(float focalx, float focaly,
    float centerx, float centery) {
    fx = focalx;
    fy = focaly;
    cx = centerx;
    cy = centery;
  };
};

struct camera_frame {
  cv::Mat image;
  float rotation;
  cv::Point3f position;
  camera_params intrinsics;

  camera_frame() {
  };

  camera_frame(const camera_frame &obj) {
    obj.image.copyTo(image);
    rotation = obj.rotation;
    position = obj.position;
    intrinsics = obj.intrinsics;
  }
};

void UpdatePosition(camera_frame &frame, 
  float distance, motion_type direction);

float getColorR(int i, int j);
float getColorG(int i, int j);
float getColorB(int i, int j);

#endif
