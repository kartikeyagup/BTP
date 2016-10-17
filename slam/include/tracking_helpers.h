#ifndef TRACKING_HELPERS_H
#define TRACKING_HELPERS_H

#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>
#include "correspondance.h"

struct img_pt {
  cv::Point2f pt;
  cv::Vec3b color;
};

struct frame_pts {
  int frame_id;
  std::unordered_map<int, img_pt> features;
};

frame_pts Track(frame_pts& init, cv::Mat &img1, cv::Mat &img2);

/**
 * @brief Inplace adds features from f2 to f1
 */
void add_more_features(frame_pts &f1, frame_pts &f2);

/**
 * @brief Compresses f1 and f2 to give compressed corr
 */
corr compress(frame_pts &f1, frame_pts &f2);

#endif
