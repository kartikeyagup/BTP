#include <iostream>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "nvmhelpers.h"

DEFINE_string(nvm_file, "data2/outputVSFM_GB.nvm", "Path to nvm file");
DEFINE_string(data_dir, "data2", "Path to directory containing images");
DEFINE_string(output_file, "data2/newVSFM_GB.nvm", "Output nvm file");
DEFINE_string(output_ply, "data2/newOutput.ply", "Generate ply file");

int main(int argc, char **argv)
{
  google::SetUsageMessage("dense --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  
  nvm_file input(FLAGS_nvm_file);
  input.save_to_disk(FLAGS_output_file);
  input.save_ply_file(FLAGS_output_ply);
  return 0;
}
