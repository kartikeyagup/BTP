#include <iostream>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "nvmhelpers.h"
#include "triangulate.h"
#include "densehelpers.h"

DEFINE_string(nvm_file, "data2/outputVSFM_GB.nvm", "Path to nvm file");
DEFINE_string(data_dir, "data2", "Path to directory containing images");
DEFINE_string(output_file, "data2/newVSFM_GB.nvm", "Output nvm file");
DEFINE_string(output_ply, "data2/newOutput.ply", "Generate ply file");
DEFINE_int32(windows, 2, "Windows size for dense matches");

int main(int argc, char **argv)
{
  google::SetUsageMessage("dense --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  
  int maxCorners = 10000;
  double qualityLevel = 0.03;
  double minDistance = 2;
  std::vector<cv::Mat> all_images;
  std::vector<cv::Mat> all_gray_images;
  std::vector<camera_frame_wo_image> camera_frame_wo_images;

  nvm_file input(FLAGS_nvm_file);
  all_images.resize(input.kf_data.size());
  all_gray_images.resize(input.kf_data.size());
  camera_frame_wo_images.resize(input.kf_data.size());

  // TODO: Add distortion parameters
  for (int i=0; i<all_images.size(); i++) {
    std::cerr << FLAGS_data_dir + "/" + input.kf_data[i].filename << "\n";
    all_images[i] = cv::imread(FLAGS_data_dir + "/" + input.kf_data[i].filename);
    cv::cvtColor(all_images[i], all_gray_images[i], CV_RGBA2GRAY);
    camera_frame_wo_images[i] = camera_frame_wo_image(
      input.kf_data[i].focal, 
      input.kf_data[i].rotation, 
      input.kf_data[i].translation,
      all_images[i].cols/2, all_images[i].rows/2);
  }

  for (int i=0; i<input.kf_data.size()-FLAGS_windows; i++) {
    // Get good points in img i
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(all_gray_images[i], corners, maxCorners, qualityLevel, minDistance);
    std::cout << "Found " << corners.size() << "\n";  
    for (auto pt: corners) {
      std::vector<triangulation_bundle> to_triangulate;
      // Get correspondances in img i+1 and i+2 ...
      to_triangulate.push_back(triangulation_bundle(camera_frame_wo_images[i], pt));
      cv::Point2f location; 
      for (int j=i+1; j<i + FLAGS_windows; j++) {
        if (findPoint(pt, all_gray_images[i], all_gray_images[j], camera_frame_wo_images[i], camera_frame_wo_images[j], location)) {
          to_triangulate.push_back(triangulation_bundle(camera_frame_wo_images[j], location));
        }
      }
      // Triangulate
      if (to_triangulate.size()>1) {
        cv::Point3f final3d = Triangulate(to_triangulate);

        // Add to input 3D cloud
        input.corr_data.push_back(Corr3D(final3d, findColor(all_images[i], pt)));  
      }
    }
  }

  input.save_to_disk(FLAGS_output_file);
  input.save_ply_file(FLAGS_output_ply);
  return 0;
}
