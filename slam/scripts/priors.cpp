#include <gflags/gflags.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include "indoorhelpers.h"
#include "nvmhelpers.h"
#include "pointcloud.h"

DEFINE_string(dir, "data5/", "Directory where nvm file is");
DEFINE_string(nvm_file, "outputVSFM_GB.nvm", "Name of nvm file");
DEFINE_int32(batch_size, 20, "Size of batch for cuboid estimation");

int main(int argc, char** argv) {
  google::SetUsageMessage("priors --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  nvm_file f(FLAGS_dir + FLAGS_nvm_file);
  f.sortNVM();
  // f.save_to_disk(FLAGS_dir + "test.nvm");
  // return 0;
  f.save_ply_file(FLAGS_dir + "raw.ply");
  std::vector<corridor> all_corridors;
  std::vector<int> motion_types;
  int st = 0, en = 2;
  for (int i = 0; i <= f.num_kf() / FLAGS_batch_size; i++) {
    all_corridors.push_back(
        corridor(f, i * FLAGS_batch_size,
                 std::min(f.num_kf(), 1 + (i + 1) * FLAGS_batch_size)));
  }
  all_corridors[0].basicInit();
  all_corridors[0].optimise_planes();
  all_corridors[0].WritePly(FLAGS_dir + "cor" + std::to_string(0) + ".ply");
  for (int i = 1; i < all_corridors.size(); i++) {
    float dotprod =
        f.getdotp((i - 1) * FLAGS_batch_size, i * (FLAGS_batch_size),
                  std::min(f.num_kf() - 1, (i + 1) * FLAGS_batch_size));
    std::cout << i << " " << dotprod << "\n";
    if (dotprod < 0.5) {
      // Turn took place
      std::cout << "Turn happened\n";
      // if (all_corridors[i - 1].ctype == straight) {
      //   std::cout << "Previous was straight\n";
      motion_types.push_back(1);
      all_corridors[i].initPlanes(
          corner, all_corridors[i - 1].plane_1, all_corridors[i - 1].plane_2,
          all_corridors[i - 1].plane_3,
          all_corridors[i - 1]
              .rotations[all_corridors[i - 1].rotations.size() - 1],
          all_corridors[i - 1]
              .trajectory[all_corridors[i - 1].trajectory.size() - 1]);
      // } else {
      //   // Previously we had corner block
      // }
    } else {
      motion_types.push_back(0);
      // Straight corridor continuing
      all_corridors[i].initPlanes(
          straight, all_corridors[i - 1].plane_1, all_corridors[i - 1].plane_2,
          all_corridors[i - 1].plane_3,
          all_corridors[i - 1]
              .rotations[all_corridors[i - 1].rotations.size() - 1],
          all_corridors[i - 1]
              .trajectory[all_corridors[i - 1].trajectory.size() - 1]);
    }
    all_corridors[i].optimise_planes();
    all_corridors[i].WritePly(FLAGS_dir + "cor" + std::to_string(i) + ".ply");
  }

  // [st, en-1] is corridor, en is new point being checked
  // while (en < f.num_kf()) {
  //   if (en - st >= FLAGS_batch_size) {
  //     all_corridors.push_back(corridor(
  //         f, st, en,
  //         FLAGS_dir + "cor" + std::to_string(all_corridors.size()) +
  //         ".ply"));
  //     motion_types.push_back(0);
  //     st = en - 1;
  //     en = en + 1;
  //   } else {
  //     Eigen::Vector3f st_prev = f.get_motion_vector(st, en - 1, st);
  //     Eigen::Vector3f prev = f.get_motion_vector(en - 1, en, st);
  //     if (fabs(prev.dot(st_prev)) < 0.4) {
  //       std::cout << "Turn took place\t" << fabs(prev.dot(st_prev)) << "\n";
  //       ;
  //       // Turn is taking place
  //       all_corridors.push_back(corridor(
  //           f, st, en,
  //           FLAGS_dir + "cor" + std::to_string(all_corridors.size()) +
  //           ".ply"));
  //       motion_types.push_back(1);
  //       st = en - 1;
  //       en = en + 1;
  //     } else {
  //       en++;
  //     }
  //   }
  // }
  // if (en - st > 10) {
  //   all_corridors.push_back(corridor(
  //       f, st, en,
  //       FLAGS_dir + "cor" + std::to_string(all_corridors.size()) + ".ply"));
  // }

  corridor cmerged = all_corridors[0];
  for (int i = 0; i < all_corridors.size(); i++) {
    all_corridors[i].WriteFile(FLAGS_dir + "corfile" + std::to_string(i) +
                               ".txt");
  }

  for (int i = 1; i < all_corridors.size(); i++) {
    fix_corridor(all_corridors[i - 1], all_corridors[i], motion_types[i - 1]);
    cmerged = add_corridor(cmerged, all_corridors[i]);
    // cmerged = merge_corridor(cmerged, all_corridors[i], motion_types[i - 1]);
  }
  // corridor c1(f, 0, 1 + (f.num_kf() / 2), "md0/test1.ply");
  // corridor c2(f, f.num_kf() / 2, f.num_kf(), "md0/test2.ply");
  // corridor cmerged = merge_corridor(c1, c2, 0);
  cmerged.WritePly(FLAGS_dir + "merged.ply");
  // f.reset_origin(0);
  f.save_ply_file(FLAGS_dir + "reset.ply");
}
