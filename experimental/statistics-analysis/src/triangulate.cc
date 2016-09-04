#include "triangulate.h"

std::vector<std::vector<cv::Point2f> > detect(cv::Mat &image) {
  std::vector<std::vector<cv::Point2f> > answer;
  for (int i=0; i<5; i++) {
    std::vector<cv::Point2f> temp;
    for(int j=0; j<5; j++) {
      temp.push_back(cv::Point2f(0,0));
    }
    answer.push_back(temp);
  }
  for (int i=0; i<5; i++) {
    for (int j=0; j<5; j++) {
      // cv::Mat tempcopy;
      // image.copyTo(tempcopy);
  
      float R, G, B;
      R = getColorR(i, j);
      G = getColorG(i, j);
      B = getColorB(i, j);
      cv::Point2f pointfound(0, 0);
      int num_points = 0;
      for (int k=0; k<image.cols; k++) {
        for (int l=0; l<image.rows; l++) {
          cv::Vec3b colorimg = image.at<cv::Vec3b>(cv::Point(k,l)); 
          if ((abs(((int)colorimg[0]) - 255*B) < 5) &&
              (abs(((int)colorimg[1]) - 255*G) < 5) &&
              (abs(((int)colorimg[2]) - 255*R) < 5)) {
            pointfound.x += k;
            pointfound.y += l;
            num_points += 1;
          }
        }
      }
      pointfound.x /= num_points;
      pointfound.y /= num_points;
      assert (num_points>0);
      // std::cout << "RGB: " << R <<"\t" << G <<"\t" << B <<"\n";
      // std::cerr << i << "\t" << j <<"\t" << num_points <<"\n";
      // cv::circle(tempcopy, pointfound, 10, cv::Scalar(0,0,255), 1, 8);
      // cv::namedWindow("Display Images");
      // cv::imshow("Display Images", tempcopy);
      // cv::waitKey(0);
      answer[i][j] = pointfound;
    }
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
  rotation[0] = cos(theta);
  rotation[1] = 0;
  rotation[2] = sin(theta);
  rotation[3] = 0;
  rotation[4] = 1;
  rotation[5] = 0;
  rotation[6] = -sin(theta);
  rotation[7] = 0;
  rotation[8] = cos(theta);
  
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

std::vector<std::vector<cv::Point3f> > detect_triangulate(
  std::vector<camera_frame> camera_frames){
  std::cout << "Entered triangulation\n";
  std::vector<std::vector<std::vector<cv::Point2f> > > all_detected;
  for (int i=0; i<camera_frames.size(); i++) {
    all_detected.push_back(detect(camera_frames[i].image));
  }
  std::vector<std::vector<cv::Point3f> > answer;

  for (int i=0; i<5; i++) {
    std::vector<cv::Point3f> answer_row;
    for (int j=0; j<5; j++) {
      std::vector<triangulation_bundle> to_triangulate;
      for (int k=0; k<all_detected.size(); k++) {
        to_triangulate.push_back(triangulation_bundle(camera_frames[k],all_detected[k][i][j]));
      }
      answer_row.push_back(Triangulate(to_triangulate));
    }
    answer.push_back(answer_row);
  }
  return answer;
}