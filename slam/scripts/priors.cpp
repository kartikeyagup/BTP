#include <gflags/gflags.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include "indoorhelpers.h"
#include "nvmhelpers.h"
#include "pointcloud.h"

DEFINE_string(nvm_file, "md0/outputVSFM_GB.nvm", "Path to nvm file");

int main(int argc, char** argv) {
  nvm_file f(FLAGS_nvm_file);

  corridor c1(f, 0, 1 + (f.num_kf() / 2), "md0/test1.ply");
  corridor c2(f, f.num_kf() / 2, f.num_kf(), "md0/test2.ply");

  corridor cmerged = merge_corridor(c1, c2);
  cmerged.WritePly("md0/merged.ply");
  // std::vector<Corr3D> requiredpts;
  // f.get_points(0, 10, requiredpts);
  // f.corr_data = requiredpts;
  // f.save_ply_file("test.ply");

  // std::vector<cv::Point3f> input;
  // input.resize(15);
  // for (size_t i = 0; i < input.size(); ++i) {
  //   input[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
  //   input[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
  //   // input[i].y = 3.0 - x;
  //   input[i].z = 2.0 - input[i].x - input[i].y;
  // }
  // // input[0].z = 2.0;
  // std::vector<int> indices;
  // plane p;
  // segment_Points(input, indices, p, 0.01);
  // std::cout << p.a << "\t" << p.b << "\t" << p.c << "\t" << p.d << "\n";
  // std::cout << indices.size() << "\n";
}
