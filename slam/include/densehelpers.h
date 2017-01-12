#ifndef DENSE_HELPERS_H
#define DENSE_HELPERS_H

#include "triangulate.h"
#include "nvmhelpers.h"

cv::Point2f makeCenterSubtracted(cv::Point2f pt, cv::Point2f center);

bool findPoint(cv::Point2f pt, 
  cv::Mat &img1,
  cv::Mat &img2,
  camera_frame_wo_image &frame_1,
  camera_frame_wo_image &frame_2,
  cv::Point2f &location);

cv::Point3i findColor(cv::Mat &img, cv::Point2f pt);

struct complete_dense {
  nvm_file nvm;
  float delta;
  std::vector<float> max_depth;
  cv::Point2f center;

  complete_dense(std::string file, std::string dirname)
    : nvm(file) {
    nvm.LoadImages(dirname);
    delta = nvm.compute_delta();
    max_depth.resize(nvm.num_frames());
    for (int i=0; i<max_depth.size(); i++) {
      max_depth[i] = nvm.compute_max_depth(i);
    }
    center = nvm.getCenter();
    std::cout << "Delta computed is " << delta << "\n";
  }

  int num_frames() {
    return nvm.num_frames();
  }

  // Col is in RGB format
  float get_discrepancy(int frame, Eigen::Vector3f p, cv::Point3i col);
  bool findNew2DPoint(int f1, int f2, cv::Point2f &p1, cv::Point2f &p2, cv::Point3f &p3d);
  bool findNew3DPoint(int f1, cv::Point2f &p1, cv::Point3f &p2, cv::Point3i &col);
  void findAll3DPoints(int framid);

  void dumpPly(std::string path) {
    nvm.save_ply_file(path);
  }

};

#endif
