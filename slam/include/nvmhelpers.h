#ifndef NVMHELPERS_H
#define NVMHELPERS_H

#include <string>
#include <vector>
#include <fstream>
#include <unordered_map>
#include <time.h>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <iostream>
#include "triangulate.h"
#include <cstddef>
#include <SiftGPU/SiftGPU.h>

struct keyframe_data {
  std::string filename;
  double focal;
  Eigen::Matrix3f rotation;
  Eigen::Vector3f translation;
  double d1;
  double d2;
};

struct imgcorr {
  int imgid;
  int siftid;
  cv::Point2f img_location;

  imgcorr() {};

  imgcorr(int id, int sft, cv::Point2f p): 
    imgid(id), siftid(sft), img_location(p) {};
};

imgcorr ChangeCamId(imgcorr inp, int x);

imgcorr fix_corr(imgcorr inp, int offset);

struct Corr3D {
  Eigen::Vector3f point_3d;
  cv::Point3i color;
  std::vector<imgcorr> corr;

  Corr3D() { };
  Corr3D(cv::Point3f p, cv::Point3i c) {
    point_3d(0, 0) = p.x;
    point_3d(1, 0) = p.y;
    point_3d(2, 0) = p.z;
    color = c; 
  }

  Corr3D(cv::Point3f p, cv::Point3i c, bool t) {
    point_3d(0, 0) = p.x;
    point_3d(1, 0) = p.y;
    point_3d(2, 0) = p.z;
    color.x = c.z;
    color.y = c.y;
    color.z = c.x;
  }

  void ChangeSiftOffset(int offset) {
    for (int i=0; i<corr.size(); i++) {
      corr[i].siftid += offset;
    }
  }

  int getSiftId() {
    assert(corr.size()>0);
    return corr[0].siftid;
  }

  int numpoints() {
    return corr.size();
  }

  bool belongs(cv::Point2f p, int framid) {
    for (auto it:corr) {
      if (it.imgid == framid) {
        cv::Point2f delta = p - it.img_location;
        return ((delta.x*delta.x + delta.y*delta.y)<25);
      }
    }
    return false;
  }

  void AddNew(cv::Point2f p, int framid) {
    int sftid = getSiftId();
    //TODO investigate
    if (!belongs(p, framid))
      corr.push_back(imgcorr(framid, sftid, p));
  }
};

Corr3D fixCorr3D(Corr3D c, int offset);
// Takes in Corr in center subtracted form only
void Triangulate_Internally(Corr3D &c, std::vector<keyframe_data> &kf_data);

struct nvm_file {
  std::string description;
  std::vector<keyframe_data> kf_data;
  std::vector<cv::Mat> images;
  std::vector<Corr3D> corr_data;
  int median_val;

  nvm_file() {};
  nvm_file(std::string path) {
    std::ifstream nvm_file;
    nvm_file.open(path);
    nvm_file >> description;
    int num_kf, num_corr;
    nvm_file >> num_kf;
    std::cerr << "Number of keyframes " << num_kf << "\n";
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
    std::cerr << "Number of correspondances " << num_corr << "\n";
    corr_data.resize(num_corr);
    std::vector<int> all_corrs_counts;
    for (int i=0; i<num_corr; i++) {
      nvm_file >> corr_data[i].point_3d(0,0) >> corr_data[i].point_3d(1,0) >> corr_data[i].point_3d(2,0)
               >> corr_data[i].color.x >> corr_data[i].color.y >> corr_data[i].color.z; 
      int num_2d;
      nvm_file >> num_2d;
      all_corrs_counts.push_back(num_2d);
      corr_data[i].corr.resize(num_2d);
      for (int j=0; j<num_2d; j++) {
        nvm_file >> corr_data[i].corr[j].imgid >> corr_data[i].corr[j].siftid
                 >> corr_data[i].corr[j].img_location.x >> corr_data[i].corr[j].img_location.y;
        if (corr_data[i].corr[j].siftid != corr_data[i].corr[0].siftid)
          std::cout << j << "\t" << corr_data[i].corr[j].siftid << "\t" << corr_data[i].corr[0].siftid << "\n";
        assert(corr_data[i].corr[j].siftid == corr_data[i].corr[0].siftid);
      }               
    }
    std::cout << "Number of corrs: " << corr_data.size() << "\n";
    nvm_file.close();
    std::nth_element(all_corrs_counts.begin(), all_corrs_counts.begin() + all_corrs_counts.size()/2, all_corrs_counts.end());
    median_val = all_corrs_counts[all_corrs_counts.size()/2];
    std::cout << "Median val: " << median_val << "\n";
  }

  static float distance(Eigen::Vector3f p1, Eigen::Vector3f p2) {
    Eigen::Vector3f diff = p1-p2;
    return sqrt(diff.dot(diff));
  }

  float compute_delta() {
    float ans = 0.0;
    int ct = 0;
    for (int i=0; i<corr_data.size(); i++) {
      for (int j=i+1; j<corr_data.size(); j++) {
        ct++;
        ans += distance(corr_data[i].point_3d, corr_data[j].point_3d);
      }
    }
    return (ans/ct)*0.1;
  }

  int get_max_sift() {
    int ans = corr_data[0].corr[0].siftid;
    for (auto it: corr_data) {
      ans = std::max(ans, it.getSiftId());
    }
    return ans;
  }

  std::vector<std::string> all_image_names(std::string &folder, std::vector<int> &frames) {
    std::vector<std::string> answer;
    for (auto it:frames) {
      answer.push_back(folder + kf_data[it].filename);
    }
    return answer;
  }

  Eigen::Vector3f GetPosition(int f1) {
    return -kf_data[f1].rotation.transpose()*kf_data[f1].translation;
  }

  std::string getFullPath(int id) {
    // TODO: Add folder name
    return kf_data[id].filename;
  }

  Eigen::Matrix3f GetIntrinsic(int f1) {
    Eigen::Matrix3f result;
    result(0, 0) = kf_data[f1].focal; result(0, 0) = 0; result(0, 0) = 0;
    result(1, 0) = 0; result(1, 0) = kf_data[f1].focal; result(1, 0) = 0;
    result(2, 0) = 0; result(2, 0) = 0; result(2, 0) = 1;
    return result;
  }

  Eigen::Matrix3f GetFundamentalMatrix(int f1, int f2) {
    Eigen::Matrix3f result;
    result = kf_data[f1].rotation.transpose()*kf_data[f2].rotation;
    Eigen::Vector3f deltac = GetPosition(f2) - GetPosition(f1);
    Eigen::Vector3f deltat = - result*deltac;
    Eigen::Matrix3f cpform;
    cpform(0,0) = 0; cpform(0,1) = -deltat(2,0); cpform(0,2) = deltat(1,0); 
    cpform(1,0) = deltat(2,0); cpform(1,1) = 0; cpform(1,2) = -deltat(0,0); 
    cpform(2,0) = -deltat(1,0); cpform(2,1) = deltat(0,0); cpform(2,2) = 0;  
    result = result*cpform;
    result = GetIntrinsic(f2).transpose()*result*GetIntrinsic(f1);
    return result;
  }

  float compute_max_depth(int id) {
    float ans = 0;
    Eigen::Vector3f cam_pos = - kf_data[id].rotation.transpose() * kf_data[id].translation;
    for (int i=0; i<corr_data.size(); i++) {
      ans = std::max(ans, distance(corr_data[i].point_3d, cam_pos));
    }
    return ans*2;
  }

  int num_frames() {
    return kf_data.size();
  }

  cv::Point2f getCenter() {
    return cv::Point2f(images[0].cols/2, images[0].rows/2);
  }

  cv::Point3i getColor(int frame, cv::Point2f p) {
    cv::Vec3b col = images[frame].at<cv::Vec3b>(p);
    return cv::Point3i((int) col[2], (int) col[1], (int) col[0]);
  }

  cv::Point3i getColorCS(int frame, cv::Point2f p) {
    cv::Vec3b col = images[frame].at<cv::Vec3b>(p + getCenter());
    return cv::Point3i((int) col[2], (int) col[1], (int) col[0]);
  }

  void LoadImages(std::string dir) {
    images.resize(kf_data.size());
    for (int i=0; i<kf_data.size(); i++) {
      images[i] = cv::imread(dir + kf_data[i].filename);
    }
  }

  void addNew3DPoint(cv::Point3f p, cv::Point3i col) {
    corr_data.push_back(Corr3D(p, col));
  }

  void NormaliseScale(float scf) {
    for (int i=0; i<kf_data.size(); i++) {
      kf_data[i].translation *= scf;
    }

    for (int i=0; i<corr_data.size(); i++) {
      corr_data[i].point_3d *= scf;
    }
  }

  void NormaliseRotation(Eigen::Matrix3f rot) {
    Eigen::Matrix3f rot_t = rot.transpose();
    for (int i=0; i<kf_data.size(); i++) {
      kf_data[i].rotation = kf_data[i].rotation*rot_t;
      kf_data[i].translation = rot * kf_data[i].translation;
    }

    for (int i=0; i<corr_data.size(); i++) {
      corr_data[i].point_3d = rot * corr_data[i].point_3d;
    } 
  }

  void NormaliseTranslation(Eigen::Vector3f trans) {
    for (int i=0; i<kf_data.size(); i++) {
      kf_data[i].translation -= trans;
    }

    for (int i=0; i<corr_data.size(); i++) {
      corr_data[i].point_3d -= trans;
    }
  }

  void NormaliseInternalRotation(int id) {
    assert(id>=0 and id<kf_data.size());
    NormaliseRotation(kf_data[id].rotation);
  }

  void NormaliseInternalTranslation(int id) {
    assert(id>=0 and id<kf_data.size());
    NormaliseTranslation(kf_data[id].translation);
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
      nvmfile << corr_data[i].point_3d(0, 0) << " " << corr_data[i].point_3d(1,0) << " " << corr_data[i].point_3d(2,0) << " "
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
            << corr_data.size() + kf_data.size() << "\nproperty float x\nproperty float y\nproperty float z\n"
            << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    for (int i=0; i<corr_data.size(); i++) {
      plyfile << corr_data[i].point_3d(0, 0) << " " << corr_data[i].point_3d(1, 0) << " " << corr_data[i].point_3d(2,0) << " " 
              << corr_data[i].color.x << " " << corr_data[i].color.y << " " << corr_data[i].color.z << "\n";
    }
    for (int i=0; i<kf_data.size(); i++) {
      Eigen::Vector3f temp = - kf_data[i].rotation.transpose() * kf_data[i].translation;
      plyfile << temp(0,0) << " " << temp(1,0) << " " << temp(2,0) << " 255 255 255\n";
    }
    plyfile.close();
  }

  void save_focal_for_optimisation(std::string path) {
    std::ofstream ffile;
    ffile.open(path);
    ffile << kf_data.size() << "\n";
    for (int i=0; i<kf_data.size(); i++) {
      ffile << kf_data[i].filename << " " << kf_data[i].focal << "\n";
    }
    ffile.close();
  }

  void save_rt_global_file(std::string path) {
    std::ofstream ffile;
    ffile.open(path);
    for (int i=0; i<kf_data.size(); i++) {
      Eigen::Vector3f temp = -kf_data[i].rotation.transpose()*kf_data[i].translation;
      ffile << kf_data[i].rotation(0,0) << " " << kf_data[i].rotation(0,1) << " " << kf_data[i].rotation(0,2) << " " 
            << kf_data[i].rotation(1,0) << " " << kf_data[i].rotation(1,1) << " " << kf_data[i].rotation(1,2) << " " 
            << kf_data[i].rotation(2,0) << " " << kf_data[i].rotation(2,1) << " " << kf_data[i].rotation(2,2) << "\n" 
            << temp(0,0) << " " << temp(1,0) << " " << temp(2,0) << "\n"; 
    }
    ffile.close();
  }

  void save_distortion_file(std::string path) {
    std::ofstream ffile;
    ffile.open(path);
    for (int i=0; i<kf_data.size(); i++) {
      ffile << kf_data[i].d1 << "\n";
    }
    ffile.close(); 
  }

  void save_invmap(std::string path) {
    std::ofstream ffile;
    ffile.open(path);
    for (int i=0; i<kf_data.size(); i++) {
      ffile << i << " " << kf_data[i].filename << "\n";
    }
    ffile.close();
  }

  void save_ours_new(std::string path) {
    std::ofstream nvmfile;
    nvmfile.open(path);
    for (int i=0; i<corr_data.size(); i++) {
      nvmfile << corr_data[i].point_3d(0, 0) << " " << corr_data[i].point_3d(1,0) << " " << corr_data[i].point_3d(2,0) << " "
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

  std::vector<std::pair<cv::Point2f, cv::Point2f> > getSiftMatches(int f1, int f2);
};

float get_best_scaling_factor(nvm_file &f1, nvm_file &f2);

void get_best_translation(nvm_file &f1, nvm_file &f2);

void GetBestRST(nvm_file &f1, nvm_file& f2);

nvm_file merge_nvm(nvm_file &f1, nvm_file &f2);

#endif
