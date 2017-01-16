#ifndef SLAM_MERGER_H
#define SLAM_MERGER_H

#include "nvmhelpers.h"
#include "sifthelpers.h"

struct slam_merger {
  std::string p1, p2;
  nvm_file f1, f2;

  slam_merger() {};
  slam_merger(std::string pat1, std::string pat2, std::string name) :
    f1(pat1+name), f2(pat2+name), p1(pat1), p2(pat2) {
    f1.LoadImages(p1);
    f2.LoadImages(p2);
  }

  void join_sets(std::vector<int> frames1, std::vector<int> frames2, 
      std::vector<Corr3D> &corr1, std::vector<Corr3D> &corr2);
  void merge_and_save(std::string path);
};

#endif
