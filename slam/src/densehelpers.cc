#include "densehelpers.h"

bool inRange(float x, float limt) {
  return (abs(x)<limt-1);
}

// x+y and x-y lie in the limt and stay positive
bool inConstrainedRange(float x, float y, float limt) {
  return ((x+y<limt-1) && (x-y>=0));
}

cv::Point2f makeCenterSubtracted(cv::Point2f pt, cv::Point2f center) {
  return pt-center;
}

float get_score(cv::Mat &img1, cv::Mat &img2, cv::Point2f p1, cv::Point2f p2) {
  int gridsize = 7;
  if (inConstrainedRange(p1.x, gridsize, img1.cols) && inConstrainedRange(p1.y, gridsize, img1.rows)) {
    if (inConstrainedRange(p2.x, gridsize, img1.cols) && inConstrainedRange(p2.y, gridsize, img2.rows)) {
      float answer = 0;
      std::vector<float> numerators (3,0);
      std::vector<float> denom1(3,0);
      std::vector<float> denom2(3,0);
      for (int i=-gridsize; i<=gridsize; i++) {
        for (int j=-gridsize; j<=gridsize; j++) {
          cv::Point2f temp(i,j);
          cv::Vec3b color1 = img1.at<cv::Vec3b>(p1+temp);
          cv::Vec3b color2 = img2.at<cv::Vec3b>(p2+temp);
          float sumc1 = color1[0] + color1[1] + color1[2];
          float sumc2 = color2[0] + color2[1] + color2[2];
          std::vector<double> c1 = {(color1[0]*1.0)/sumc1, (color1[1]*1.0)/sumc1, (color1[2]*1.0)/sumc1};
          std::vector<double> c2 = {(color2[0]*1.0)/sumc2, (color2[1]*1.0)/sumc2, (color2[2]*1.0)/sumc2};
          numerators[0] += c1[0]*c2[0];
          numerators[1] += c1[1]*c2[1];
          numerators[2] += c1[2]*c2[2];
          denom1[0] += c1[0]*c1[0];
          denom1[1] += c1[1]*c1[1];
          denom1[2] += c1[2]*c1[2];
          denom2[0] += c2[0]*c2[0];
          denom2[1] += c2[1]*c2[1];
          denom2[2] += c2[2]*c2[2];


          float diff_1 = color1[0] - color2[0];
          float diff_2 = color1[1] - color2[1];
          float diff_3 = color1[2] - color2[2]; 
          answer += (diff_1*diff_1) + (diff_2*diff_2) + (diff_3*diff_3);
          
        }
      }
      numerators[0] /= sqrt(denom1[0]*denom2[0]);
      numerators[1] /= sqrt(denom1[1]*denom2[1]);
      numerators[2] /= sqrt(denom1[2]*denom2[2]);
      // std::cout << numerators[0] << "\t" << numerators[1] << "\t" << numerators[2] <<"\n";
      // answer += numerators[0]*numerators[0];
      // answer += numerators[1]*numerators[1];
      // answer += numerators[2]*numerators[2];

       // + numerators[1] + numerators[2];
      // answer /= 255.0;
      answer /= (1+gridsize)*(1+gridsize)*3;
      // std::cerr << answer << "\n";
      return answer;
    }
  }
  return 10000;
}

// Takes RGB Image
bool findPoint(cv::Point2f pt, 
  cv::Mat &img1,
  cv::Mat &img2,
  camera_frame_wo_image &frame_1,
  camera_frame_wo_image &frame_2,
  cv::Point2f &location) {

  // cv::Mat img1_c = img1.clone();
  // cv::Mat img2_c = img2.clone();
  Eigen::Matrix3f tx;
  tx.setZero();
  cv::Point3f temp1 = frame_2.position - frame_1.position;
  Eigen::Vector3f temp;
  temp(0, 0) = temp1.x;
  temp(1, 0) = temp1.y;
  temp(2, 0) = temp1.z;

  temp = frame_1.rotation*temp;
  Eigen::Matrix3f r = frame_2.rotation * frame_1.rotation.transpose();
  tx(0,1) = - temp(2, 0);
  tx(0,2) =   temp(1, 0);
  tx(1,0) =   temp(2, 0);
  tx(1,2) = - temp(0, 0);
  tx(2,0) = - temp(1, 0);
  tx(2,1) =   temp(0, 0);
  Eigen::Matrix3f prod = tx*r;
  Eigen::Vector3f p1;
  cv::Point2f center(img1.cols/2, img1.rows/2);
  std::vector<cv::Point2f> corners;
  pt -= center;
  p1(0, 0) = pt.x / frame_1.intrinsics.f;
  p1(1, 0) = pt.y / frame_1.intrinsics.f;
  p1(2, 0) = 1;
  Eigen::Vector3f line = prod*p1;
  line(2, 0)*= frame_2.intrinsics.f;

  if (fabs(line(0,0))<1e-8) {
    // std::cerr << "Ill conditioned x\n";
    return false;
  }
  if (fabs(line(1,0))<1e-8) {
    // std::cerr << "Ill conditioned y\n";
    return false;
  }

  if (inRange((-line(2,0)-line(1,0)*center.y)/line(0,0), center.x)) {
    corners.push_back(cv::Point2f((-line(2,0)-line(1,0)*center.y)/line(0,0), center.y));
  }
  if (inRange((-line(2,0)+line(1,0)*center.y)/line(0,0), center.x)) {
    corners.push_back(cv::Point2f((-line(2,0)+line(1,0)*center.y)/line(0,0), -center.y));
  }
  if (inRange((-line(2,0)-line(0,0)*center.x)/line(1,0), center.y)) {
    corners.push_back(cv::Point2f(center.x, (-line(2,0)-line(0,0)*center.x)/line(1,0)));
  }
  if (inRange((-line(2,0)+line(0,0)*center.x)/line(1,0), center.y)) {
    corners.push_back(cv::Point2f(-center.x, (-line(2,0)+line(0,0)*center.x)/line(1,0)));
  }
  assert(corners.size()<=2);
  if (corners.size()<2) {
    // std::cerr << "Only 1 point in the frame\n";
    return false;
  }

  assert(corners.size()==2);
  for (int i=0; i<corners.size(); i++) {
    corners[i] += center;
  }
  pt += center;
  
  float bestsofar = 10000;
  cv::Point found;
  for (int i=0; i<200; i++) {
    for (int j=-5; j<=5; j++) {
      cv::Point2f linept = (i*corners[0] + (200-i)*corners[1])/200;
      linept.y += j;
      // cv::circle(img2_c, linept, 5, cv::Scalar(255,0,0), -1);
      float score = get_score(img1, img2, pt, linept);
      if (score<bestsofar) {
        // std::cerr << score << "\n";
        bestsofar = score;
        found = linept;
      }
    }
  }

  if (bestsofar>1000) {
    // std::cerr << "Could not find a good point: " << bestsofar << "\n";
    return false;
  } else {
    location = found;
    // std::cout << bestsofar << "\n";
    // cv::namedWindow("new", 0);
    // cv::namedWindow("orig", 1);
    // cv::circle(img1_c, pt, 10, cv::Scalar(0), -1);
    // cv::circle(img2_c, corners[0], 10, cv::Scalar(0,0,255), -1);
    // cv::circle(img2_c, corners[1], 10, cv::Scalar(0,0,255), -1);
    // cv::circle(img2_c, found, 10, cv::Scalar(255,0,0), -1);
    // cv::line(img2_c, corners[0], corners[1], cv::Scalar(0));
    // cv::imshow("orig", img1_c);
    // cv::imshow("new", img2_c);
    // while (cv::waitKey(1) != 27) {
    // }

    return true;
  }

  assert(false);
  return true;
}

cv::Point3i findColor(cv::Mat &img, cv::Point2f pt) {
  cv::Point3i answer;
  cv::Vec3b color = img.at<cv::Vec3b>(cv::Point(pt.x, pt.y));
  answer.x = color[0];
  answer.y = color[1];
  answer.z = color[2];
  // Find color
  return answer;
}

float complete_dense::get_discrepancy(int frame, Eigen::Vector3f p, cv::Point3i col, cv::Point2f &imgpt) {
  // Project ray from p to frame.
  // Obtain color at the point if it is in the frame
  // Return the discrepancy from expected color
  Eigen::Vector3f pos = nvm.kf_data[frame].rotation*p + nvm.kf_data[frame].translation;
  pos(0, 0) *= nvm.kf_data[frame].focal;
  pos(1, 0) *= nvm.kf_data[frame].focal;
  imgpt.x = pos(0,0)/pos(2,0); 
  imgpt.y = pos(1,0)/pos(2,0);
  imgpt += center;
  if (imgpt.x<0 || imgpt.x>=2*center.x || imgpt.y<0 || imgpt.y>=2*center.y) {
    return 10000;
  } else {
    cv::Vec3b color = nvm.images[frame].at<cv::Vec3b> (imgpt);
    float ans = 0.0;
    ans += abs(((int) color[0]) - col.z);
    ans += abs(((int) color[1]) - col.y);
    ans += abs(((int) color[2]) - col.x);
    return ans;
  }
}

float complete_dense::findNew2DPoint(int f1, int f2, cv::Point2f &p1, cv::Point2f &p2, Eigen::Vector3f &p3d) {
  // Generate ray direction from f1 frame and p1 point on image
  // Traverse the ray in intervals of delta till you reach the max depth distance
  // If discrepancy at a point is minimum and is also below a threshold, then it is the 3D point
  Eigen::Vector3f dir;
  cv::Point2f centersub = p1 - center;
  dir(0, 0) = centersub.x;
  dir(1, 0) = centersub.y;
  dir(2, 0) = nvm.kf_data[f1].focal;
  dir = nvm.kf_data[f1].rotation.transpose()*dir;
  dir /= sqrt(dir.dot(dir));
  dir *= delta;
  Eigen::Vector3f cent = -nvm.kf_data[f1].rotation.transpose()*nvm.kf_data[f1].translation;
  cv::Point3i color = nvm.getColor(f1, p1);
  float bestdisc = 1000;
  
  // cv::Mat temp, temp1, temp2;
  // nvm.images[f1].copyTo(temp1);
  // nvm.images[f2].copyTo(temp);
  for (int i=0; i<max_depth[f1]; i++) {
    float newdesc = get_discrepancy(f2, cent, color, p2);
    // if (newdesc<10000) {
    //   cv::circle(temp, p2, 5, cv::Scalar(255,0,0), -1);
    // }
    if (newdesc < bestdisc) {
      // std::cout << "Optimised to " << newdesc << "\t" << cent << "\n";
      // nvm.images[f2].copyTo(temp2);  
      // cv::circle(temp2, p2, 5, cv::Scalar(0,255,0), -1);
  
      bestdisc = newdesc;
      p3d(0, 0) = cent(0, 0);
      p3d(1, 0) = cent(1, 0);
      p3d(2, 0) = cent(2, 0);
    } 
    cent += dir;
  }
  // cv::circle(temp1, p1, 5, cv::Scalar(0,0,255),-1);
  // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
  // cv::imshow( "Display window", temp);                   // Show our image inside it.
  // cv::namedWindow( "Display window1", cv::WINDOW_AUTOSIZE );// Create a window for display.
  // cv::imshow( "Display window1", temp1);                   // Show our image inside it.
  // cv::namedWindow( "Display window2", cv::WINDOW_AUTOSIZE );// Create a window for display.
  // cv::imshow( "Display window2", temp2);                   // Show our image inside it.
  // cv::waitKey(0);

  return bestdisc;
}

bool complete_dense::findNew3DPoint(int f1, cv::Point2f p1, cv::Point3f &p2) {
  // Call find new2d Point on different image sets and return the 3D output point and color
  Eigen::Vector3f result, temp;
  cv::Point2f p2d;
  float best = 1000;
  for (int i=0; i<num_frames(); i++) {
    if (abs(i-f1)>10) {
      float quality = findNew2DPoint(f1, i, p1, p2d, temp);
      if (quality < best) {
        best = quality;
        p2.x = temp(0, 0);
        p2.y = temp(1, 0);
        p2.z = temp(2, 0);
      }
    }
  }
  if (best>5)
    return false;
  Eigen::Vector3f pteigen;
  pteigen(0, 0) = p2.x;
  pteigen(1, 0) = p2.y;
  pteigen(2, 0) = p2.z;
  cv::Point3i color = nvm.getColor(f1, p1);
  cv::Point2f useless;
  
  int posvoting(0);
  int negvoting(0);
  for (int i=0; i<nvm.kf_data.size(); i++) {
    if (get_discrepancy(i, pteigen, color, useless) < 5)
      posvoting++;
    else
      negvoting++;
  }
  return (posvoting>10);
}

void complete_dense::findAll3DPoints(int framid) {
  // Run the find new 3D point over all points in a frame id
  nvm.corr_data.clear();
  for (int i=0; i<center.x; i++) {
    for (int j=0; j<center.y; j++) {
      std::cout << "\rFinding for point " << framid << "\t" <<  2*i << ", " << 2*j << ":" << nvm.corr_data.size() << std::flush;  
      cv::Point3f p3new;
      if (findNew3DPoint(framid, cv::Point2f(2*i, 2*j), p3new)) {
        nvm.addNew3DPoint(p3new, nvm.getColor(framid, cv::Point2f(2*i, 2*j)));
      }
    }
  }
}
