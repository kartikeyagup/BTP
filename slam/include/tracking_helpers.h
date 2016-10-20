#ifndef TRACKING_HELPERS_H
#define TRACKING_HELPERS_H

#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>
#include <algorithm>
#include <utility>
#include "correspondance.h"

struct img_pt {
  cv::Point2f pt;
  cv::Vec3b color;

  img_pt() {};
  img_pt(cv::Point2f p, cv::Vec3b v) {
    pt = p;
    color = v;
  };

};

bool compare_my(const std::pair<int, cv::Point2f> &p1, const std::pair<int, cv::Point2f> &p2);
struct frame_pts {
  int frame_id;
  std::unordered_map<int, img_pt> features;

  frame_pts() {};
  frame_pts(int x) {
    frame_id = x;
  }

  std::vector<cv::Point2f> get_vector(std::vector<int> &siftids) {
    std::vector<cv::Point2f> answer;
    siftids.clear();
    std::vector<std::pair<int, cv::Point2f> > tempv;
    for (auto it: features) {
      tempv.push_back(std::make_pair(it.first, it.second.pt));
    }
    // std::sort(tempv.begin(), tempv.end(), compare_my);
    for (auto it: tempv) {
      answer.push_back(it.second);
      siftids.push_back(it.first);
    }
    assert(siftids.size() == answer.size());
    return answer;
  }
};

frame_pts Track(frame_pts& init, cv::Mat &img1, cv::Mat &img2, int fid, int cx, int cy);

/**
 * @brief Inplace adds features from f2 to f1
 */
void add_more_features(frame_pts &f1, frame_pts &f2);

/**
 * @brief Compresses f1 and f2 to give compressed corr
 */
corr compress(frame_pts &f1, frame_pts &f2);

bool WithinCompressionRange(frame_pts &f1, frame_pts &f2);

#endif
