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
  // [st, en-1] is corridor, en is new point being checked
  while (en < f.num_kf()) {
    if (en - st >= FLAGS_batch_size) {
      all_corridors.push_back(corridor(
          f, st, en,
          FLAGS_dir + "cor" + std::to_string(all_corridors.size()) + ".ply"));
      motion_types.push_back(0);
      st = en - 1;
      en = en + 1;
    } else {
      Eigen::Vector3f st_prev = f.get_motion_vector(st, en - 1, st);
      Eigen::Vector3f prev = f.get_motion_vector(en - 1, en, st);
      if (fabs(prev.dot(st_prev)) < 0.4) {
        std::cout << "Turn took place\t" << fabs(prev.dot(st_prev)) << "\n";
        ;
        // Turn is taking place
        all_corridors.push_back(corridor(
            f, st, en,
            FLAGS_dir + "cor" + std::to_string(all_corridors.size()) + ".ply"));
        motion_types.push_back(1);
        st = en - 1;
        en = en + 1;
      } else {
        en++;
      }
    }
  }
  if (en - st > 10) {
    all_corridors.push_back(corridor(
        f, st, en,
        FLAGS_dir + "cor" + std::to_string(all_corridors.size()) + ".ply"));
  }

  corridor cmerged = all_corridors[0];
  for (int i = 1; i < all_corridors.size(); i++) {
    cmerged = merge_corridor(cmerged, all_corridors[i], motion_types[i - 1]);
  }
  // corridor c1(f, 0, 1 + (f.num_kf() / 2), "md0/test1.ply");
  // corridor c2(f, f.num_kf() / 2, f.num_kf(), "md0/test2.ply");
  // corridor cmerged = merge_corridor(c1, c2, 0);
  cmerged.WritePly(FLAGS_dir + "merged.ply");
  f.reset_origin(0);
  f.save_ply_file(FLAGS_dir + "reset.ply");
}
