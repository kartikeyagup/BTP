#include "triangulate.h"

cv::Point3f Triangulate(std::vector<triangulation_bundle> &input) {
  int num_eqs = 2*input.size();
  int num_vars = 3;
  cv::Mat A(num_eqs, 3, CV_32F);
  cv::Mat B(num_eqs, 1, CV_32F);
  cv::Mat X(3, 1, CV_32F);
  for (int i=0; i<input.size(); i++) {
    float *rotation = new float[9];
    rotation[0] = input[i].camera.rotation(0,0);
    rotation[1] = input[i].camera.rotation(0,1);
    rotation[2] = input[i].camera.rotation(0,2);
    rotation[3] = input[i].camera.rotation(1,0);
    rotation[4] = input[i].camera.rotation(1,1);
    rotation[5] = input[i].camera.rotation(1,2);
    rotation[6] = input[i].camera.rotation(2,0);
    rotation[7] = input[i].camera.rotation(2,1);
    rotation[8] = input[i].camera.rotation(2,2);
  
    float *translation = new float[3];
    // translation[0] = rotation[0]*input[i].camera.position.x + 
    //   rotation[1]*input[i].camera.position.y +
    //   rotation[2]*input[i].camera.position.z;
    // translation[1] = rotation[3]*input[i].camera.position.x + 
    //   rotation[4]*input[i].camera.position.y +
    //   rotation[5]*input[i].camera.position.z;
    // translation[2] = rotation[6]*input[i].camera.position.x + 
    //   rotation[7]*input[i].camera.position.y +
    //   rotation[8]*input[i].camera.position.z;
    // translation[0] *= -1;
    // translation[1] *= -1;
    // translation[2] *= -1;
    // translation = - RC

    translation[0] = input[i].camera.position.x;
    translation[1] = input[i].camera.position.y;
    translation[2] = input[i].camera.position.z;

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
    delete rotation;
  }
  cv::solve(A, B, X, cv::DECOMP_SVD);
  cv::Point3f answer;
  answer.x = X.at<float>(0,0);
  answer.y = X.at<float>(1,0);
  answer.z = X.at<float>(2,0);
  return answer;
}
