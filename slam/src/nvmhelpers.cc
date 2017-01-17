#include "nvmhelpers.h"

imgcorr ChangeCamId(imgcorr inp, int x) {
  inp.imgid = x;
  return inp;
}

imgcorr fix_corr(imgcorr inp, int offset) {
  imgcorr result;

  result.imgid = inp.imgid + offset;
  result.siftid = inp.siftid;
  result.img_location = inp.img_location;

  return result;
}

Corr3D fixCorr3D(Corr3D c, int offset) {
  Corr3D ans;
  ans.point_3d = c.point_3d;
  ans.color = c.color;
  for (int i=0; i<c.corr.size(); i++) {
    ans.corr.push_back(fix_corr(c.corr[i], offset));
  }
  return ans;
}

void Triangulate_Internally(Corr3D &c, std::vector<keyframe_data> &kf_data) {
  assert(c.corr.size()>1);
  std::vector<triangulation_bundle> to_triangulate;

  for (auto it: c.corr) {
    triangulation_bundle t(
        camera_frame_wo_image(kf_data[it.imgid].focal, 
            kf_data[it.imgid].rotation, 
            kf_data[it.imgid].translation), 
        it.img_location);
    to_triangulate.push_back(t);
  }

  cv::Point3f result = Triangulate(to_triangulate);
  c.point_3d(0, 0) = result.x;
  c.point_3d(1, 0) = result.y;
  c.point_3d(2, 0) = result.z;
}

std::vector<std::pair<cv::Point2f, cv::Point2f> > nvm_file::getSiftMatches(int f1, int f2) {
  std::vector<std::pair<cv::Point2f, cv::Point2f> > answer;
  std::vector<std::pair<cv::Point2f, cv::Point2f> > temp = RunSift(getFullPath(f1), getFullPath(f2));
  Eigen::Matrix3f fmat = GetFundamentalMatrix(f1, f2);
  for (auto it: temp) {
    Eigen::Vector3f xdash, x;
    xdash(0,0) = it.second.x;
    xdash(1,0) = it.second.y;
    xdash(2,0) = 1;
    x(0,0) = it.first.x;
    x(1,0) = it.first.y;
    x(2,0) = 1;
  }
  return answer;
}

float get_best_scaling_factor(nvm_file &f1, nvm_file &f2) {
  std::unordered_map<int, Eigen::Vector3f> m1;
  for (auto it: f1.corr_data) {
    m1[it.corr[0].siftid] = it.point_3d;
  }
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > points_common;

  for (auto it: f2.corr_data) {
    if (m1.find(it.corr[0].siftid) != m1.end()) {
      // std::cout << it.corr[0].siftid << "\n";
      // std::cout << m1[it.corr[0].siftid] << "\n";
      points_common.push_back(std::make_pair(m1[it.corr[0].siftid], it.point_3d));
    }
  }
  srand(time(NULL));
  float d1(0), d2(0);
  int limt = points_common.size();
  for (int i=0; i<500; i++) {
    int p1 = rand()%limt;
    int p2 = rand()%limt;
    // std::cout << p1 << "\t" << p2 <<"\n";
    // std::cout << points_common[p1].first << "\t" << points_common[p2].first << "\n";
    d1 += (points_common[p1].first - points_common[p2].first).norm();
    d2 += (points_common[p1].second - points_common[p2].second).norm();
  }
  std::cerr << "Number of common points " << points_common.size() << "\n";
  std::cerr << d1 << "\t" << d2 << "\t" << d1/d2 << "\n";
  return d1/d2;
}

void get_best_translation(nvm_file &f1, nvm_file &f2) {
  std::unordered_map<int, Eigen::Vector3f> m1;
  for (auto it: f1.corr_data) {
    m1[it.corr[0].siftid] = it.point_3d;
  }
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > points_common;

  for (auto it: f2.corr_data) {
    if (m1.find(it.corr[0].siftid) != m1.end()) {
      // std::cout << it.corr[0].siftid << "\n";
      // std::cout << m1[it.corr[0].siftid] << "\n";
      points_common.push_back(std::make_pair(m1[it.corr[0].siftid], it.point_3d));
    }
  }

  Eigen::Vector3f answer1, answer2;
  answer1.setZero();
  answer2.setZero();

  for (auto it: points_common) {
    answer1 += it.first;
    answer2 += it.second;
  }
  answer1(0,0) /= points_common.size();
  answer1(1,0) /= points_common.size();
  answer1(2,0) /= points_common.size();

  answer2(0,0) /= points_common.size();
  answer2(1,0) /= points_common.size();
  answer2(2,0) /= points_common.size();

  std::cerr << answer1 << "\n";
  std::cerr << answer2 << "\n";

  for (Corr3D &it: f1.corr_data) {
    it.point_3d -= answer1;
    // it.color.x=255;
    // it.color.y=0;
    // it.color.z=0;
  }
  for (Corr3D &it: f2.corr_data) {
    it.point_3d -= answer2;
    // it.color.x=0;
    // it.color.y=0;
    // it.color.z=255;
  }
  // std::cout << answer << "\n";
  // return answer;
}

void GetBestRST(nvm_file &f1, nvm_file& f2) {
  std::unordered_map<int, Eigen::Vector3f> m1;
  for (auto it: f1.corr_data) {
    if (it.corr.size()>=0)
      m1[it.corr[0].siftid] = it.point_3d;
  }
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > points_common;

  for (auto it: f2.corr_data) {
    if (it.corr.size()>=0) {
      if (m1.find(it.corr[0].siftid) != m1.end()) {
        points_common.push_back(std::make_pair(m1[it.corr[0].siftid], it.point_3d));
      }
    }
  }

  std::cout << "Number of points " << points_common.size() << "\n";

  Eigen::Vector3f answer1, answer2;
  answer1.setZero();
  answer2.setZero();

  for (auto it: points_common) {
    answer1 += it.first;
    answer2 += it.second;
  }
  answer1(0,0) /= points_common.size();
  answer1(1,0) /= points_common.size();
  answer1(2,0) /= points_common.size();

  answer2(0,0) /= points_common.size();
  answer2(1,0) /= points_common.size();
  answer2(2,0) /= points_common.size();

  Eigen::MatrixXf points1 = Eigen::MatrixXf(3, points_common.size());
  Eigen::MatrixXf points2 = Eigen::MatrixXf(3, points_common.size());

  for (int i=0; i<points_common.size(); i++) {
    points1(0,i) = points_common[i].first(0,0) - answer1(0,0);
    points1(1,i) = points_common[i].first(1,0) - answer1(1,0);
    points1(2,i) = points_common[i].first(2,0) - answer1(2,0);    
    points2(0,i) = points_common[i].second(0,0) - answer2(0,0);
    points2(1,i) = points_common[i].second(1,0) - answer2(1,0);
    points2(2,i) = points_common[i].second(2,0) - answer2(2,0);
  }

  Eigen::MatrixXf r(Eigen::MatrixXf::Identity(3,3));
  float scale = 1;
  Eigen::Vector3f t;
  t.setZero();
  if (points_common.size()>=3) {
    Eigen::MatrixXf temp = points1 * (points2.transpose());
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(temp, Eigen::ComputeThinU | Eigen::ComputeThinV);
  
    Eigen::MatrixXf s = Eigen::MatrixXf::Identity(3, 3);
    r = svd.matrixU() * svd.matrixV().transpose();
    if (r.determinant() < 0) {
      int minsofar = 0;
      float minval = r(0,0);
      if (r(1,1)<minval) {
        minval = r(1,1);
        minsofar = 1;
      }
      if (r(2,2)<minval) {
        minval = r(2,2);
        minsofar = 2;
      }
      s(minsofar, minsofar) = -1;
      r = svd.matrixU() * s *  svd.matrixV().transpose();
    }
  
    scale = s(0,0)*svd.singularValues()(0,0) + 
        s(1,1)*svd.singularValues()(1,0) + 
        s(2,2)*svd.singularValues()(2,0); 
  
    float denom = points2.squaredNorm();
    scale /= denom;
  
    t= answer1 - scale*r*answer2;
  }

  // FOR HARDCODING ONLY
  // scale = 16.0;
  // t << 0.137, -0.145, 0.86;
  // r << 0.6431,         0,   -0.7658,
  //     -0.1065,    0.9903,   -0.0895,
  //      0.7583,    0.1391,    0.6369;

  std::cout << "scale: " << scale << "\n";
  std::cout << "rotation " << r << "\n";
  std::cout << "trans " << t << "\n";
  for (int i=0; i<f2.corr_data.size(); i++) {
    f2.corr_data[i].point_3d = scale*r*f2.corr_data[i].point_3d + t;
  }

  for (int i=0; i<f2.kf_data.size(); i++) {
    f2.kf_data[i].rotation = f2.kf_data[i].rotation * r.transpose();
    f2.kf_data[i].translation = scale*f2.kf_data[i].translation - f2.kf_data[i].rotation*t;
  }
}

nvm_file merge_nvm(nvm_file &f1, nvm_file &f2) {
  nvm_file output;
  assert(f1.description == f2.description);
  output.description = f1.description;
  for (int i=0; i<f1.kf_data.size(); i++) {
    output.kf_data.push_back(f1.kf_data[i]);
  }
  int offset = 0;
  if (f1.kf_data.rbegin()->filename == f2.kf_data.begin()->filename) {
    offset = 1;
  }
  for (int i=offset; i<f2.kf_data.size(); i++) {
    output.kf_data.push_back(f2.kf_data[i]);
  }

  std::unordered_map<int, Corr3D> all_mappings;
  for (int i=0; i<f1.corr_data.size(); i++) {
    all_mappings[f1.corr_data[i].corr[0].siftid] = f1.corr_data[i];
  }
  offset = f1.kf_data.size()-offset;

  for (int i=0; i<f2.corr_data.size(); i++) {
    int sftid = f2.corr_data[i].corr[0].siftid;
    if (all_mappings.find(sftid) == all_mappings.end()) {
      all_mappings[sftid].point_3d = f2.corr_data[i].point_3d;
      all_mappings[sftid].color = f2.corr_data[i].color;  
      for (int j=0; j<f2.corr_data[i].corr.size(); j++) {
        all_mappings[sftid].corr.push_back(fix_corr(f2.corr_data[i].corr[j], offset));
      }
    } else {
      // sift id existed
      for (int j=0; j<f2.corr_data[i].corr.size(); j++) {
        if (f2.corr_data[i].corr[j].imgid>0) {
          all_mappings[sftid].corr.push_back(fix_corr(f2.corr_data[i].corr[j], offset));
        }
      }
    }
  }

  for (auto it: all_mappings) {
    output.corr_data.push_back(it.second);
  }  
  std::cout << "Number of points " << output.corr_data.size() << "\n";

  return output;
}
