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
};

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
    return ans/ct;
  }

  float compute_max_depth(int id) {
    float ans = 0;
    Eigen::Vector3f cam_pos = - kf_data[id].rotation.transpose() * kf_data[id].translation;
    for (int i=0; i<corr_data.size(); i++) {
      ans = std::max(ans, distance(corr_data[i].point_3d, cam_pos));
    }
    return ans;
  }

  int num_frames() {
    return kf_data.size();
  }

  void LoadImages(std::string dir) {
    images.resize(kf_data.size());
    for (int i=0; i<kf_data.size(); i++) {
      images[i] = cv::imread(dir + kf_data[i].filename);
    }
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
};

float get_best_scaling_factor(nvm_file &f1, nvm_file &f2);

void get_best_translation(nvm_file &f1, nvm_file &f2);

void GetBestRST(nvm_file &f1, nvm_file& f2);

nvm_file merge_nvm(nvm_file &f1, nvm_file &f2);

#endif
