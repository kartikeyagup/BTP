#include "statistics.h"

void dump_disk(std::unordered_map<TwoDPoint, std::pair<cv::Point2f, cv::Point3f> > inputpoints,
  grid_params grid_description,
  motion_type motion,
  float angle,
  int num_images,
  float distance,
  camera_params intrinsics,
  cv::Point3f starting_point,
  std::vector<camera_frame> &cam_frames,
  std::string dump_directory,
  bool dump_images) {

  std::string mk_command = "mkdir " + dump_directory;
  int result = system(mk_command.c_str());
  assert(result==0);
  
  // Writing grid
  std::ofstream gridfile;
  gridfile.open(dump_directory + "/grid.txt");
  for (auto it : inputpoints) {
    gridfile << it.first.x << "\t" << it.first.y << "\t" 
             << it.second.first << "\t" 
             << it.second.second <<"\n";
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
  // num_images
  infofile << num_images <<"\n";
  infofile.close();

  if (dump_images) {
    // Saving images
    for (int i=0; i<cam_frames.size(); i++) {
      cv::imwrite(dump_directory+"/image_"+std::to_string(i)+".png", cam_frames[i].image); 
    }
  }
};
