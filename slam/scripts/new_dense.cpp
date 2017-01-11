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

  cd.dumpPly(FLAGS_dirname + FLAGS_output);
  return 0;
}
