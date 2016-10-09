#ifndef NVMHELPERS_H
#define NVMHELPERS_H

#include <string>
#include <vector>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

struct keyframe_data {
  std::string filename;
  double focal;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  double d1;
  double d2;
};

struct imgcorr {
  int imgid;
  int siftid;
  cv::Point2d img_location;
};

struct Corr3D {
  cv::Point3d point_3d;
  cv::Point3i color;
  std::vector<imgcorr> corr;
};

struct nvm_file {
  std::string description;
  std::vector<keyframe_data> kf_data;
  std::vector<Corr3D> corr_data;

  nvm_file();
  nvm_file(std::string path) {
    std::ifstream nvm_file;
    nvm_file.open(path);
    nvm_file >> description;
    int num_kf, num_corr;
    nvm_file >> num_kf;
    kf_data.resize(num_kf);
    for (int i=0; i<num_kf; i++) {
      nvm_file >> kf_data[i].filename >> kf_data[i].focal 
               >> kf_data[i].rotation(0,0) >> kf_data[i].rotation(0,1) >> kf_data[i].rotation(0,2)
               >> kf_data[i].rotation(1,0) >> kf_data[i].rotation(1,1) >> kf_data[i].rotation(1,2)
               >> kf_data[i].rotation(2,0) >> kf_data[i].rotation(2,1) >> kf_data[i].rotation(2,2)
               >> kf_data[i].translation(0,0) >> kf_data[i].translation(1,0) >> kf_data[i].translation(2,0) 
               >> kf_data[i].d1 >> kf_data[i].d2;
    }
    nvm_file >> num_corr;
    corr_data.resize(num_corr);
    for (int i=0; i<num_corr; i++) {
      nvm_file >> corr_data[i].point_3d.x >> corr_data[i].point_3d.y >> corr_data[i].point_3d.z
               >> corr_data[i].color.x >> corr_data[i].color.y >> corr_data[i].color.z; 
      int num_2d;
      nvm_file >> num_2d;
      corr_data[i].corr.resize(num_2d);
      for (int j=0; j<num_2d; j++) {
        nvm_file >> corr_data[i].corr[j].imgid >> corr_data[i].corr[j].siftid
                 >> corr_data[i].corr[j].img_location.x >> corr_data[i].corr[j].img_location.y;
      }               
    }
    nvm_file.close();
  }

  void save_to_disk(std::string path) {
    std::ofstream nvmfile;
    nvmfile.open(path);
    nvmfile.precision(8);
    nvmfile << description << "\n";
    nvmfile << kf_data.size() << "\n";
    for (int i=0; i<kf_data.size(); i++) {
      nvmfile << kf_data[i].filename << " "
              << kf_data[i].focal << " "
              << kf_data[i].rotation(0,0) << " " << kf_data[i].rotation(0,1) << " " << kf_data[i].rotation(0,2) << " "
              << kf_data[i].rotation(1,0) << " " << kf_data[i].rotation(1,1) << " " << kf_data[i].rotation(1,2) << " "
              << kf_data[i].rotation(2,0) << " " << kf_data[i].rotation(2,1) << " " << kf_data[i].rotation(2,2) << " "
              << kf_data[i].translation(0,0) << " " << kf_data[i].translation(1,0) << " " << kf_data[i].translation(2,0) << " "
              << kf_data[i].d1 << " " << kf_data[i].d2 << "\n";
    }
    nvmfile << corr_data.size() << "\n";
    for (int i=0; i<corr_data.size(); i++) {
      nvmfile << corr_data[i].point_3d.x << " " << corr_data[i].point_3d.y << " " << corr_data[i].point_3d.z 
              << corr_data[i].color.x << " " << corr_data[i].color.y << " " << corr_data[i].color.z << " "
              << corr_data[i].corr.size() << " ";
      for (int j=0; j<corr_data[i].corr.size(); j++) {
        nvmfile << corr_data[i].corr[j].imgid << " "
                << corr_data[i].corr[j].siftid << " "
                << corr_data[i].corr[j].img_location.x << " " << corr_data[i].corr[j].img_location.y << " ";
      }
      nvmfile << "\n";
    }
    nvmfile.close();
  }

  void save_ply_file(std::string path) {
    std::ofstream plyfile;
    plyfile.open(path);
    plyfile.precision(8);
    plyfile << "ply\nformat ascii 1.0\nelement vertex "
            << corr_data.size() << "\nproperty float x\nproperty float y\nproperty float z\n"
            << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    for (int i=0; i<corr_data.size(); i++) {
      plyfile << corr_data[i].point_3d.x << " " << corr_data[i].point_3d.y << " " << corr_data[i].point_3d.z << " " 
              << corr_data[i].color.x << " " << corr_data[i].color.y << " " << corr_data[i].color.z << "\n";
    }
    plyfile.close();
  }
};

#endif
