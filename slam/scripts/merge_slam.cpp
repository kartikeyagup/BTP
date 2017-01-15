#include <gflags/gflags.h>
#include "slam_merger.h"

DEFINE_string(path_1, "lab/data0/", "Path of first data");
DEFINE_string(path_2, "lab/data1/", "Path of first data");
DEFINE_string(nvm_file, "outputVSFM_GB.nvm", "Name of nvm file");
DEFINE_string(output_dir, "lab/merged/", "Path of merged directory");

int main(int argc, char **argv) {
  google::SetUsageMessage("big_merge --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  slam_merger s(FLAGS_path_1, FLAGS_path_2, FLAGS_nvm_file);
  s.merge_and_save(FLAGS_output_dir);
  return 0;
}
