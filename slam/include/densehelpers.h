#ifndef DENSE_HELPERS_H
#define DENSE_HELPERS_H

#include "triangulate.h"

bool findPoint(cv::Point2f pt, 
  cv::Mat &img1,
  cv::Mat &img2,
  camera_frame_wo_image &frame_1,
  camera_frame_wo_image &frame_2,
  cv::Point2f &location);

cv::Point3i findColor(cv::Mat &img, cv::Point2f pt);

#endif
