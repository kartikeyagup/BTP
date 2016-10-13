#include <iostream>
#include "nvmhelpers.h"
#include <gflags/gflags.h>
#include <string>

DEFINE_string(path1, "btp/batch0/sfm.nvm", "Path to 1st nvm file");
DEFINE_string(path2, "btp/batch1/sfm.nvm", "Path to 2nd nvm file");
DEFINE_string(output_ply, "btp/combined01.ply", "Output combined ply file");
DEFINE_string(output_nvm, "btp/combined01.nvm", "Output combined nvm file");

int main(int argc, char **argv)
{
  google::SetUsageMessage("dense --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  
  nvm_file f1(FLAGS_path1);
  std::cerr << "Done reading 1\n";
  nvm_file f2(FLAGS_path2);
  std::cerr << "Done reading 2\n";

  // f2.NormaliseScale(get_best_scaling_factor(f1, f2));
  // f2.NormaliseScale(get_best_scaling_factor(f1, f2));

  // f2.NormaliseInternalTranslation(0);

  // f2.NormaliseTranslation(-f1.kf_data.rbegin()->translation);
  // f2.NormaliseTranslation(get_best_translation(f1, f2));
  // get_best_translation(f1, f2);
  // get_best_translation(f1, f2);
  // f2.NormaliseInternalRotation(0);
  // f1.NormaliseInternalRotation(f1.kf_data.size()-1);
  // f2.NormaliseRotation(f1.kf_data.rbegin()->rotation.transpose());

  GetBestRST(f1, f2);

  for (auto it: f2.kf_data) {
    f1.kf_data.push_back(it);
  }  

  for (auto it: f2.corr_data) {
    f1.corr_data.push_back(it);
  }

  f1.save_to_disk(FLAGS_output_nvm);
  f1.save_ply_file(FLAGS_output_ply);

  return 0;
}
