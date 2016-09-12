#include "triangulate.h"

std::unordered_map<TwoDPoint, cv::Point2f> detect(
  grid_params &grid_description,
  cv::Mat &image) {
  std::unordered_map<TwoDPoint, cv::Point2f> answer;

  std::unordered_map<Color, std::pair<cv::Point2f, int> > detection;
  for (int k=0; k<image.cols; k++) {
    for (int l=0; l<image.rows; l++) {
      cv::Vec3b colorimg = image.at<cv::Vec3b>(cv::Point(k,l)); 
      Color newcol((int)colorimg[2], (int)colorimg[1], int(colorimg[0]));
      if (newcol.isblack()) {
        continue;
      }
      auto got = detection.find(newcol);
      if (got == detection.end()) {
        // New color
        detection[newcol] = std::make_pair(cv::Point2f(k, l) , 1);
      } else {
        // Old color
        detection[newcol].first.x += k;
        detection[newcol].first.y += l;
        detection[newcol].second += 1;
      }
    }
  }

  // Associate colors
  for (auto kv : detection) {
    // find color i j
    auto ij = grid_description.c2p.find(kv.first);
    assert (ij != grid_description.c2p.end());
    answer[grid_description.c2p[kv.first]]=
      cv::Point2f(kv.second.first.x/kv.second.second,
                  kv.second.first.y/kv.second.second);
  }

  return answer;
}
  
void ConvertPoint(triangulation_bundle &bundle) {
  bundle.pt.x -= bundle.camera.intrinsics.cx;
  bundle.pt.y -= bundle.camera.intrinsics.cy;
  bundle.pt.x /= bundle.camera.intrinsics.f;
  bundle.pt.y /= bundle.camera.intrinsics.f; 
}

cv::Point3f Triangulate(std::vector<triangulation_bundle> &input) {
  for (int i=0; i<input.size(); i++) {
    ConvertPoint(input[i]);
  }
  float theta = input[0].camera.rotation;
  float *rotation = new float[9];
  rotation[0] = cos(theta*PI/180);
  rotation[1] = 0;
  rotation[2] = sin(theta*PI/180);
  rotation[3] = 0;
  rotation[4] = 1;
  rotation[5] = 0;
  rotation[6] = -sin(theta*PI/180);
  rotation[7] = 0;
  rotation[8] = cos(theta*PI/180);
  
  rotation[2] *= -1;
  rotation[5] *= -1;
  rotation[8] *= -1;

  int num_eqs = 2*input.size();
  int num_vars = 3;
  cv::Mat A(num_eqs, 3, CV_32F);
  cv::Mat B(num_eqs, 1, CV_32F);
  cv::Mat X(3, 1, CV_32F);
  for (int i=0; i<input.size(); i++) {
    float *translation = new float[3];
    translation[0] = rotation[0]*input[i].camera.position.x + 
     rotation[1]*input[i].camera.position.y +
     rotation[2]*input[i].camera.position.z;
    translation[1] = rotation[3]*input[i].camera.position.x + 
     rotation[4]*input[i].camera.position.y +
     rotation[5]*input[i].camera.position.z;
    translation[2] = rotation[6]*input[i].camera.position.x + 
     rotation[7]*input[i].camera.position.y +
     rotation[8]*input[i].camera.position.z;
    translation[0] *= -1;
    translation[1] *= -1;
    translation[2] *= -1;
    // translation = - RC
    int row = 2*i;

    A.at<float>(row, 0) = rotation[0] - input[i].pt.x*rotation[6];
    A.at<float>(row, 1) = rotation[1] - input[i].pt.x*rotation[7];
    A.at<float>(row, 2) = rotation[2] - input[i].pt.x*rotation[8];
    
    A.at<float>(row + 1, 0) = rotation[3] - input[i].pt.y*rotation[6];
    A.at<float>(row + 1, 1) = rotation[4] - input[i].pt.y*rotation[7];
    A.at<float>(row + 1, 2) = rotation[5] - input[i].pt.y*rotation[8];
    
    B.at<float>(row + 0, 0) = translation[2]*input[i].pt.x - translation[0];
    B.at<float>(row + 1, 0) = translation[2]*input[i].pt.y - translation[1];
    delete translation;
  }
  cv::solve(A, B, X, cv::DECOMP_SVD);
  delete rotation;
  cv::Point3f answer;
  answer.x = X.at<float>(0,0);
  answer.y = X.at<float>(1,0);
  answer.z = X.at<float>(2,0);
  return answer;
}

std::unordered_map<TwoDPoint, std::pair<cv::Point2f, cv::Point3f> > detect_triangulate(
  grid_params &grid_description,
  std::vector<camera_frame> camera_frames) {

  std::vector<std::vector<std::pair<TwoDPoint, cv::Point2f> > > all_detected;
  std::unordered_map<TwoDPoint, std::pair<cv::Point2f, cv::Point3f> > answer;
  std::unordered_map<TwoDPoint, std::vector<triangulation_bundle> > bundles;
   
  for (int i=0; i<camera_frames.size(); i++) {
    std::unordered_map<TwoDPoint, cv::Point2f> frame_detect = 
      detect(grid_description, camera_frames[i].image);
    for (auto fm : frame_detect) {
      if (bundles.find(fm.first) == bundles.end()) {
        // New 2D point
        std::vector<triangulation_bundle> bundle;
        bundle.push_back(triangulation_bundle(camera_frames[i],fm.second));
        bundles[fm.first] = bundle;
      } else {
        triangulation_bundle temp(camera_frames[i], fm.second);
        bundles[fm.first].push_back(temp);
      }
    }  
  }

  std::cerr << "Done with detections\n";

  for (auto bd: bundles) {
    if (bd.second.size()>1) {
      answer[bd.first] = std::make_pair(bd.second[0].pt*bd.second[0].camera.intrinsics.f, Triangulate(bd.second));
    }
  }
  return answer;
}
