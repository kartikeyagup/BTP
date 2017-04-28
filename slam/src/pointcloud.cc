#include "pointcloud.h"
#include <unordered_set>

void segment_Points(std::vector<cv::Point3f> &inputpoints,
                    std::vector<int> &in_inliers, plane &p, float distance, bool side) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Fill in the cloud data
  cloud->width = inputpoints.size();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate the data
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = inputpoints[i].x;
    cloud->points[i].y = inputpoints[i].y;
    cloud->points[i].z = inputpoints[i].z;
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  if (side) {
    std::cout << "Entered\n";
    Eigen::Vector3f axis;
    axis.setZero();
    axis(0,0) = -1.0;
    seg.setAxis(axis);
    seg.setEpsAngle(10.0f*(M_PI)/180.0);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  } else {
    seg.setModelType(pcl::SACMODEL_PLANE);
  }
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }
  p.a = coefficients->values[0];
  p.b = coefficients->values[1];
  p.c = coefficients->values[2];
  p.d = coefficients->values[3];

  in_inliers.resize(0);
  for (int i = 0; i < inliers->indices.size(); i++) {
    in_inliers.push_back(inliers->indices[i]);
  }
}

void segment_Points_with_ref(std::vector<cv::Point3f> &inputpoints,
                    std::vector<int> &in_inliers, plane &p, float distance, const Eigen::Vector3f & ref) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Fill in the cloud data
  cloud->width = inputpoints.size();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate the data
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = inputpoints[i].x;
    cloud->points[i].y = inputpoints[i].y;
    cloud->points[i].z = inputpoints[i].z;
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setEpsAngle(5.0f * (M_PI/180.0f)); 
  seg.setAxis(ref);
  seg.setDistanceThreshold(distance);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  in_inliers.resize(0);
  
  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }
  p.a = coefficients->values[0];
  p.b = coefficients->values[1];
  p.c = coefficients->values[2];
  p.d = coefficients->values[3];

  for (int i = 0; i < inliers->indices.size(); i++) {
    in_inliers.push_back(inliers->indices[i]);
  }
}

std::vector<cv::Point3f> filterPoints(std::vector<cv::Point3f> &input,
                                      std::vector<int> &inliers,
                                      std::vector<cv::Point3f> &valid) {
  std::vector<cv::Point3f> answer;
  valid.clear();
  std::unordered_set<int> frames;
  for (auto it : inliers) {
    valid.push_back(input[it]);
    frames.insert(it);
  }
  for (int i = 0; i < input.size(); i++) {
    if (frames.find(i) == frames.end()) {
      answer.push_back(input[i]);
    }
  }
  return answer;
}

void fitPlane(std::vector<cv::Point3f> &inpoints,
              std::vector<cv::Point3f> &planepts, plane &p, float dist, bool side) {
  std::vector<int> inliers1;
  segment_Points(inpoints, inliers1, p, dist, side);
  std::vector<cv::Point3f> remaining_pts =
      filterPoints(inpoints, inliers1, planepts);
  inpoints = remaining_pts;
}

void fit3Planes(std::vector<cv::Point3f> &inputpoints,
                std::vector<cv::Point3f> &plane1,
                std::vector<cv::Point3f> &plane2,
                std::vector<cv::Point3f> &plane3, plane &p1, plane &p2,
                plane &p3, float distance) {
  std::vector<int> inliers1, inliers2, inliers3;
  segment_Points(inputpoints, inliers1, p1, distance);
  std::vector<cv::Point3f> remaining_pts =
      filterPoints(inputpoints, inliers1, plane1);

  segment_Points(remaining_pts, inliers2, p2, distance);
  std::vector<cv::Point3f> remaining_pts_1 =
      filterPoints(remaining_pts, inliers2, plane2);

  segment_Points(remaining_pts_1, inliers3, p3, distance);
  std::vector<cv::Point3f> remaining_pts2 =
      filterPoints(remaining_pts_1, inliers3, plane3);
}

std::vector<std::vector<int> > fit3Planes_with_inliers(std::vector<cv::Point3f> &inputpoints,
                std::vector<cv::Point3f> &plane1,
                std::vector<cv::Point3f> &plane2,
                std::vector<cv::Point3f> &plane3, plane &p1, plane &p2,
                plane &p3, float distance) {
  std::vector<int> inliers1, inliers2, inliers3;
  segment_Points(inputpoints, inliers1, p1, distance);
  std::vector<cv::Point3f> remaining_pts =
      filterPoints(inputpoints, inliers1, plane1);

  segment_Points(remaining_pts, inliers2, p2, distance);
  std::vector<cv::Point3f> remaining_pts_1 =
      filterPoints(remaining_pts, inliers2, plane2);

  segment_Points(remaining_pts_1, inliers3, p3, distance);
  std::vector<cv::Point3f> remaining_pts2 =
      filterPoints(remaining_pts_1, inliers3, plane3);

  std::vector<std::vector<int> > answer;
  answer.push_back(inliers1);
  answer.push_back(inliers2);
  answer.push_back(inliers3);
  return answer;
}


void fit3Planes_with_ref(std::vector<cv::Point3f> &inputpoints,
                std::vector<cv::Point3f> &plane1,
                std::vector<cv::Point3f> &plane2,
                std::vector<cv::Point3f> &plane3,
                std::vector<cv::Point3f> &plane4, plane &p1, plane &p2,
                plane &p3, plane &p4, float distance){
  
  //Assuming roof points have already been removed

  std::vector<int> inliers2, inliers3, inliers4;
  // segment_Points(inputpoints, inliers1, p1, distance);
  // std::vector<cv::Point3f> remaining_pts =
  //   filterPoints(inputpoints, inliers1, plane1);

  segment_Points(inputpoints, inliers2, p2, distance);
  std::vector<cv::Point3f> remaining_pts_2 =
    filterPoints(inputpoints, inliers2, plane2);

  const Eigen::Vector3f dir(p2.a, p2.b, p2.c);

  segment_Points_with_ref(remaining_pts_2, inliers3, p3, distance, dir);
  std::vector<cv::Point3f> remaining_pts_3 =
    filterPoints(remaining_pts_2, inliers3, plane3);

  std::cout<<"Inliers size: "<<inliers3.size()<<" Rem points:"<<remaining_pts_2.size()<<std::endl;
  segment_Points_with_ref(remaining_pts_3, inliers4, p4, distance, dir);
  std::vector<cv::Point3f> remaining_pts_4 =
    filterPoints(remaining_pts_3, inliers4, plane4);
  std::cout<<"Inliers size: "<<inliers3.size()<<" Rem points:"<<remaining_pts_3.size()<<std::endl;

  std::cout<<p2.a<<" "<<p2.b<<" "<<p2.c<<std::endl;
  std::cout<<p3.a<<" "<<p3.b<<" "<<p3.c<<std::endl;

  std::cout<<p2.dotp(p3)<<" "<<p2.dotp(p4)<<std::endl;


}