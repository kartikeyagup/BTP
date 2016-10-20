#include "tracking_helpers.h"
#include <iostream>

bool compare_my(const std::pair<int, cv::Point2f> &p1, const std::pair<int, cv::Point2f> &p2) {
  return (p1.first >= p2.first);
}

frame_pts Track(frame_pts& init, cv::Mat &oldFrame, cv::Mat &newFrame, int fid,
  int cx, int cy) {
  frame_pts answer(fid);

  std::vector<cv::Point2f> corners_prev, corners, corners_inverse;
  std::vector<uchar> status, status_inverse;
  std::vector<float> err;
  cv::Size winSize(15, 15);
  cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.03);
  std::vector<int> siftids;
  corners_prev = init.get_vector(siftids);

  calcOpticalFlowPyrLK(oldFrame, newFrame,
        corners_prev, corners, status,
        err, winSize, 2, termcrit, 0, 0.001);

  calcOpticalFlowPyrLK(newFrame, oldFrame,
        corners, corners_inverse, status_inverse,
        err, winSize, 2, termcrit, 0, 0.001);
      
  GetGoodPoints(corners_prev,corners_inverse,status,status_inverse);

  for (int i=0; i<corners.size(); i++) {
    if (status[i]) {
      if ((corners[i].x >= (2*cx -1)) || (corners[i].y >= (2*cy -1)) || (corners[i].x <0) || (corners[i].y<0))
      {
        status[i] = 0;
        continue;
      }
      assert(corners[i].x < 2*cx-1);
      assert(corners[i].y < 2*cy-1);
      assert(corners[i].x >= 0);
      assert(corners[i].y >= 0);
      assert(answer.features.find(siftids[i])==answer.features.end());
      answer.features[siftids[i]] = img_pt(corners[i], init.features[siftids[i]].color);
    }
  }
  return answer;
}

void add_more_features(frame_pts &f1, frame_pts &f2) {
  assert(f1.frame_id == f2.frame_id);
  for (auto it: f2.features) {
    if (f1.features.find(it.first) == f1.features.end()) {
      f1.features[it.first] = it.second;
    }
  }
}

corr compress(frame_pts &f1, frame_pts &f2) {
  assert(f1.frame_id != f2.frame_id);
  corr answer(f1.frame_id, f2.frame_id);

  for (auto it: f2.features) {
    if (f1.features.find(it.first) != f1.features.end()) {
      answer.unique_id.push_back(it.first);
      answer.p1.push_back(f1.features[it.first].pt);
      answer.p2.push_back(f2.features[it.first].pt);
      answer.col.push_back(f1.features[it.first].color);
    }
  }

  return answer;
}

bool WithinCompressionRange(frame_pts &f1, frame_pts&f2) {
  cv::Point2f res;

  int ct = 0;
  for (auto it: f2.features) {
    if (f1.features.find(it.first)!= f1.features.end()) {
      res += it.second.pt - f1.features[it.first].pt;
      ct ++;
    }
  }
  res.x /= ct;
  res.y /= ct;

  float val = res.dot(res);
  return (val < 500);
}
