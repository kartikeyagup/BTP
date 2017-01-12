#include <iostream>
#include "densehelpers.h"
#include <gflags/gflags.h>

DEFINE_string(nvm_file, "outputVSFM_GB.nvm", "Name of nvm file");
DEFINE_string(dirname, "new5point/img/", "Directory containing nvm file and images");
DEFINE_string(output, "DenseTry.ply", "Name of ply file to be generated");

int main(int argc, char **argv) {
  google::SetUsageMessage("new_dense --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  complete_dense cd(FLAGS_dirname + FLAGS_nvm_file, FLAGS_dirname);

  for (int i=0; i<cd.num_frames(); i++) {
    cd.findAll3DPoints(i);
  }

  std::cout << cd.nvm.corr_data[0].corr[0].img_location << "\n";
  std::cout << cd.get_discrepancy(cd.nvm.corr_data[0].corr[0].imgid, cd.nvm.corr_data[0].point_3d, cd.nvm.corr_data[0].color) << "\n";

  cd.dumpPly(FLAGS_dirname + FLAGS_output);
  return 0;
}
