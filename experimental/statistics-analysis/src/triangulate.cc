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

cv::Point3f Triangulate(std::vector<triangulation_bundle> &input) {
  cv::Point3f answer;
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