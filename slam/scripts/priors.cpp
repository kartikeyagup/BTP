#include <gflags/gflags.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include "indoorhelpers.h"
#include "nvmhelpers.h"
#include "pointcloud.h"

DEFINE_string(dir, "data5/", "Directory where nvm file is");
DEFINE_string(nvm_file, "outputVSFM_GB.nvm", "Name of nvm file");
DEFINE_int32(batch_size, 20, "Size of batch for cuboid estimation");
DEFINE_int32(reset_pt, 0, "Point till which the coordinate axis is reset");

int main(int argc, char** argv) {
  google::SetUsageMessage("priors --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  nvm_file f(FLAGS_dir + FLAGS_nvm_file);
  f.sortNVM();
  f.save_ply_file(FLAGS_dir + "raw.ply");
  f.reset_origin(0);
  Eigen::Vector3f srcv;
  srcv.setZero();
  srcv(2, 0) = 1.0;
  Eigen::Vector3f destv = f.get_translation_vector(0, 20);
  Eigen::Matrix3f rotreq = GetRotMatrix(destv, srcv);
  f.rotate(rotreq.transpose(), rotreq);
  std::vector<corridor> all_corridors;
  std::vector<int> motion_types;
  int st = 0, en = 2;
  for (int i = 0; i <= std::min(9, f.num_kf() / FLAGS_batch_size);
       i++) {  // f.num_kf() / FLAGS_batch_size; i++) {
    all_corridors.push_back(
        corridor(f, i * FLAGS_batch_size,
                 std::min(f.num_kf(), 1 + (i + 1) * FLAGS_batch_size)));
  }
  if (FLAGS_reset_pt > 0) {
    all_corridors[0].basicInit(1);
    all_corridors[0].WritePly(FLAGS_dir + "cor" + std::to_string(0) + ".ply");
    return 0;
  }
  all_corridors[0].basicInit();
  all_corridors[0].optimise_planes();
  all_corridors[0].WritePly(FLAGS_dir + "cor" + std::to_string(0) + ".ply");
  motion_types.push_back(0);
  for (int i = 1; i < all_corridors.size(); i++) {
    float dotprod =
        f.getdotp((i - 1) * FLAGS_batch_size, i * (FLAGS_batch_size),
                  std::min(f.num_kf() - 1, (i + 1) * FLAGS_batch_size));
    Eigen::Vector3f prevv = f.get_motion_vector((i - 1) * FLAGS_batch_size,
                                                i * (FLAGS_batch_size), 0);
    Eigen::Vector3f curv = f.get_motion_vector(
        (i)*FLAGS_batch_size,
        std::min(f.num_kf() - 1, (i + 1) * FLAGS_batch_size), 0);
    std::cout << i << " " << dotprod << " v norm " << curv(1, 0) * curv(1, 0)
              << "\n";
    // if (curv(1, 0) * curv(1, 0) > 0.01) {
    //   std::cout << "Vertical Motion took place\n";
    // }
    bool vmotion = (curv(1, 0) * (curv(1, 0)) > 0.01);
    if (!vmotion) {
      if (dotprod < 0.5 && motion_types.back() >= 0) {
        // Turn took place
        std::cout << "Turn happened\n";
        motion_types.push_back(1);
        all_corridors[i].initPlanes(
            corner, all_corridors[i - 1].planes[0],
            all_corridors[i - 1].planes[1], all_corridors[i - 1].planes[2],
            all_corridors[i - 1]
                .rotations[all_corridors[i - 1].rotations.size() - 1],
            all_corridors[i - 1]
                .trajectory[all_corridors[i - 1].trajectory.size() - 1]);
      } else if (dotprod > 0.8 && motion_types.back() >= 0) {
        motion_types.push_back(0);
        // Straight corridor continuing
        all_corridors[i].initPlanes(
            straight, all_corridors[i - 1].planes[0],
            all_corridors[i - 1].planes[1], all_corridors[i - 1].planes[2],
            all_corridors[i - 1]
                .rotations[all_corridors[i - 1].rotations.size() - 1],
            all_corridors[i - 1]
                .trajectory[all_corridors[i - 1].trajectory.size() - 1]);
      } else {
        std::cout << "Regular ambigous\n";
        all_corridors[i].basicInit(-1);
        motion_types.push_back(-100);
      }
    } else {
      if (motion_types.back() >= 0 or motion_types.back() == -100) {
        // Init stairs, fit 2 planes
        all_corridors[i].basicInit(-1);
        std::cout << "Done a basic init of the stairs\n";
        all_corridors[i].optimise_planes();
        motion_types.push_back(-1);
      } else if (motion_types.back() == -1 && dotprod > 0.8) {
        // Straight upstairs
        std::cout << "Going straight upstairs\n";
        all_corridors[i].initPlanes(
            upstairs, all_corridors[i - 1].planes[0],
            all_corridors[i - 1].planes[1], all_corridors[i - 1].planes[2],
            all_corridors[i - 1]
                .rotations[all_corridors[i - 1].rotations.size() - 1],
            all_corridors[i - 1]
                .trajectory[all_corridors[i - 1].trajectory.size() - 1]);
        all_corridors[i].optimise_planes();
        motion_types.push_back(-1);
      } else if (motion_types.back() == -1 && dotprod < 0.5) {
        // Turn upstairs
        std::cout << "Turning upstairs\n";
        motion_types.push_back(-2);
      } else {
        // ambigous upstairs
        std::cout << "Ambigous upstairs\n";
        motion_types.push_back(-100);
      }
    }
    // all_corridors[i].optimise_planes();
    all_corridors[i].WritePly(FLAGS_dir + "cor" + std::to_string(i) + ".ply");
  }

  corridor cmerged = all_corridors[0];
  // for (int i = 0; i < all_corridors.size(); i++) {
  //   all_corridors[i].WriteFile(FLAGS_dir + "corfile" + std::to_string(i) +
  //                              ".txt");
  // }

  for (int i = 1; i < all_corridors.size(); i++) {
    fix_corridor(all_corridors[i - 1], all_corridors[i], motion_types[i]);
    cmerged = add_corridor(cmerged, all_corridors[i]);
  }
  cmerged.WritePly(FLAGS_dir + "merged.ply");
  f.save_ply_file(FLAGS_dir + "reset.ply");
}
