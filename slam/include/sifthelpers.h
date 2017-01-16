#ifndef SIFT_HELPERS_H
#define SIFT_HELPERS_H

#include <cstddef>
#include <SiftGPU/SiftGPU.h>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <opencv2/core/core.hpp>

struct sift_corr{
  std::vector<SiftGPU::SiftKeypoint> first_img, second_img;

  std::vector<std::pair<cv::Point2f, cv::Point2f> > GetCSCorrs(cv::Point2f center) {
    std::vector<std::pair<cv::Point2f, cv::Point2f> > answer;
    assert(first_img.size()==second_img.size());
    for (int i=0; i<first_img.size(); i++) {
      answer.push_back(
        std::make_pair(
          cv::Point2f(first_img[i].x, first_img[i].y) - center,
          cv::Point2f(first_img[i].x, first_img[i].y) - center));
    }
    return answer;
  }
};

struct sift_point{
  int x;
  int y;

  bool operator==(const sift_point &other) const { 
    return (x == other.x&& y == other.y);
  }  
};

#endif
namespace std {
  template<>
  struct hash<sift_point> {
    std::size_t operator()(const sift_point& k) const {
      using std::size_t;
      using std::hash;
      using std::string;
  
      return ((hash<int>()(k.x)
             ^ (hash<int>()(k.y) << 1)) >> 1);
    }
  };
}

struct sift_corr Running_SIFT(std::string file1, std::string file2);
std::vector<std::pair<cv::Point2f, cv::Point2f> > RunSift(std::string f1, std::string f2, cv::Point2f center);
