#include "statistics.h"

void dump_disk(std::vector<std::vector<cv::Point3f> > inputpoints,
  grid_params grid_description,
  motion_type motion,
  float angle,
  int num_images,
  float distance,
  camera_params intrinsics,
  cv::Point3f starting_point,
  std::vector<camera_frame> &cam_frames,
  std::string dump_directory) {

  std::string mk_command = "mkdir " + dump_directory;
  int result = system(mk_command.c_str());
  assert(result==0);
  std::cerr << "Created output directory\n";

  // Writing grid
  std::ofstream gridfile;
  gridfile.open(dump_directory + "/grid.txt");
  for (int i=0; i<inputpoints.size(); i++) {
    for (int j=0; j<inputpoints[i].size(); j++) {
      gridfile << inputpoints[i][j];
      gridfile << ";";
    }
    gridfile << "\n";
  }
  gridfile.close();

  // Writing additional info
  std::ofstream infofile;
  infofile.open(dump_directory + "/info.txt");
  // grid specs
  infofile << grid_description.gridx << "\t"
           << grid_description.gridy << "\n";
  // angle
  infofile << angle <<"\n";
  // motion direction
  infofile << motion << "\n";
  // distance
  infofile << distance << "\n";
  // intrinsics
  infofile << intrinsics.f  << "\t"
           << intrinsics.cx << "\t"
           << intrinsics.cy << "\n";
  // starting pt
  infofile << starting_point << "\n";
  infofile.close();

  // Saving images
  for (int i=0; i<cam_frames.size(); i++) {
    cv::imwrite(dump_directory+"/image_"+std::to_string(i)+".png", cam_frames[i].image); 
  }
};
