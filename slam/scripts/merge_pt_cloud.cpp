#include <iostream>
#include "nvmhelpers.h"
#include <gflags/gflags.h>
#include <string>

DEFINE_string(path1, "btp/batch2/", "Path to dir of 1st nvm file");
DEFINE_string(path2, "btp/batch3/", "Path to dir of 2nd nvm file");
DEFINE_string(output_dir, "btp/", "Output combined ply file");

int main(int argc, char **argv)
{
  google::SetUsageMessage("dense --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  
  nvm_file f1(FLAGS_path1 + "outputVSFM_GB.nvm");
  std::cerr << "Done reading 1\n";
  nvm_file f2(FLAGS_path2 + "outputVSFM_GB.nvm");
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

  // for (auto it: f2.kf_data) {
  //   f1.kf_data.push_back(it);
  // }  

  // for (auto it: f2.corr_data) {
  //   f1.corr_data.push_back(it);
  // }
  nvm_file merged = merge_nvm(f1, f2);

  merged.save_to_disk(FLAGS_output_dir + "combined.nvm");
  merged.save_ply_file(FLAGS_output_dir + "combined.ply");

  merged.save_focal_for_optimisation(FLAGS_output_dir + "focal_for_optimisation.txt");
  merged.save_rt_global_file(FLAGS_output_dir + "RTglobalmapped_for_optimisation.txt");
  merged.save_distortion_file(FLAGS_output_dir + "Distortion.txt");
  merged.save_ours_new(FLAGS_output_dir+"ours_new.txt");
  merged.save_invmap(FLAGS_output_dir+"Invmap.txt");

  return 0;
}
