#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <gflags/gflags.h>

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
  float f;
  float cx;
  float cy;

  camera_params() { };

  camera_params(float focal,
    float centerx, float centery) {
    f = focal;
    cx = centerx;
    cy = centery;
  };

  camera_params(const camera_params &obj) {
    f = obj.f;
    cx = obj.cx;
    cy = obj.cy;
  }
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

struct triangulation_bundle {
  camera_frame camera;
  cv::Point2f pt;

  triangulation_bundle() { };

  triangulation_bundle(camera_frame cam, 
    cv::Point2f point) {
    camera = cam;
    pt = point;
  }
};

void UpdatePosition(camera_frame &frame, 
  float distance, motion_type direction);

float getColorR(int i, int j);
float getColorG(int i, int j);
float getColorB(int i, int j);

#endif
