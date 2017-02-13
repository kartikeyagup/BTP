#ifndef PLYFILE_H
#define PLYFILE_H
#include <vector>
#include <utility>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <fstream>

struct plyfile {
  std::vector<std::pair<cv::Point3d, cv::Point3i> > alldata;

  plyfile() {};
  plyfile(std::string path) {
    std::ifstream ply_file;
    ply_file.open(path);
    int numcors;
    ply_file >> numcors;
    alldata.resize(numcors);
    for (int i=0; i<numcors; i++) {
      ply_file >> alldata[i].first.x
               >> alldata[i].first.y
               >> alldata[i].first.z
               >> alldata[i].second.x
               >> alldata[i].second.y
               >> alldata[i].second.z;
    }
  }

  int NumPoints() {
    return alldata.size();
  }
};

#endif
