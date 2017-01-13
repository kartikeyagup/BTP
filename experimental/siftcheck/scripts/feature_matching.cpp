#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <SiftGPU/SiftGPU.h>
#include <string>
#include <iostream>
#include <vector>
#include <nvmhelpers.h>
#include <triangulate.h>
#include <typeinfo>
using namespace cv;
using namespace std;

string output_path = "/home/vision/Desktop/btp_data/output/";

struct sift_corr{
  std::vector<SiftGPU::SiftKeypoint> first_img;
    std::vector<SiftGPU::SiftKeypoint> second_img;
};

struct Point_Frame{
  int x;
  int y;
  string frame_id;

  bool operator==(const Point_Frame &other) const
    { return (x == other.x
            && y == other.y
            && frame_id == other.frame_id);
  }  
};

struct sift_point{
  int x;
  int y;

  bool operator==(const sift_point &other) const
    { return (x == other.x
            && y == other.y);
  }  
};

namespace std {

  template <>
  struct hash<Point_Frame>
  {
    std::size_t operator()(const Point_Frame& k) const
    {
      using std::size_t;
      using std::hash;
      using std::string;

      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:

      return ((hash<int>()(k.x)
               ^ (hash<int>()(k.y) << 1)) >> 1)
               ^ (hash<string>()(k.frame_id) << 1);
    }
  };

  template<>
  struct hash<sift_point>
  {
    std::size_t operator()(const sift_point& k) const
    {
      using std::size_t;
      using std::hash;
      using std::string;

      return ((hash<int>()(k.x)
               ^ (hash<int>()(k.y) << 1)) >> 1);
    }
  };

}



struct RST{
  float scale;
  Eigen::MatrixXf r;
  Eigen::Vector3f t;
  RST(){
    scale = 1.0;
    r = Eigen::MatrixXf::Identity(3,3);
    t.setZero();
  }
};
// global variables

int siftid = 0;

unordered_map<int, vector<Point_Frame>> siftid_to_point_img1;
unordered_map<int, vector<Point_Frame>> siftid_to_point_img2;

unordered_map<Point_Frame,int> point_to_siftid_img1;
unordered_map<Point_Frame,int> point_to_siftid_img2;

unordered_map<string,keyframe_data> first_file;
unordered_map<string,keyframe_data> second_file;

unordered_map<string,int> position_first;
unordered_map<string,int> position_second;








struct sift_corr Running_SIFT(string file1, string file2){
    SiftGPU sift;
    char * test [] = {"-fo","-1","-da","-v","0","-p","5184x3456","-b","-nomc"};
    sift.ParseParam(9,test);

    int support  = sift.CreateContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    {
      struct sift_corr temp;
      return temp;
    } 
    sift.RunSIFT(file1.c_str());
    // sift.SaveSIFT("img1.sift");
    int num1 = sift.GetFeatureNum();


    std::vector<float> des1(128*num1);
    std::vector<SiftGPU::SiftKeypoint> keys1(num1);

    sift.GetFeatureVector(&keys1[0], &des1[0]);

    sift.RunSIFT(file2.c_str());
    // sift.SaveSIFT("img2.sift");
    int num2 = sift.GetFeatureNum();

    cout << "number of matches   : "<< num1 << "  " << num2 << endl;

    std::vector<float> des2(128*num2);
    std::vector<SiftGPU::SiftKeypoint>keys2(num2);
    sift.GetFeatureVector(&keys2[0], &des2[0]);

    SiftMatchGPU matcher(4096);
    matcher.VerifyContextGL();

    matcher.SetDescriptors(0, num1, &des1[0]); //image 1
    matcher.SetDescriptors(1, num2, &des2[0]); //image 2

    int (*match_buf)[2] = new int[num1][2];

    int num_match = matcher.GetSiftMatch(num1, match_buf);

    // std::cout << num_match << "   sift matches were found;\n";

    struct sift_corr answer;

    unordered_map<struct sift_point, bool> first_img;
    unordered_map<struct sift_point, bool> second_img;

    for(int i = 0 ; i < num_match ; i++)
    {
        SiftGPU::SiftKeypoint & key1 = keys1[match_buf[i][0]];//feature point of first image
        SiftGPU::SiftKeypoint & key2 = keys2[match_buf[i][1]];//corresponding feature point of second image
        struct sift_point f1;
        f1.x = (int)key1.x;
        f1.y = (int)key1.y;
        struct sift_point f2;
        f2.x = (int)key2.x;
        f2.y = (int)key2.y;
        if(first_img.count(f1) == 0 && second_img.count(f2) == 0){
          answer.first_img.push_back(keys1[match_buf[i][0]]);
          answer.second_img.push_back(keys2[match_buf[i][1]]);
          first_img[f1] = true;
          second_img[f2] = true;
        }
        // cout << key1.x << "   " << key2.x << "   " << key1.y << "   " << key2.y << "\n";
    }

    return answer;
  }

struct RST GetBestRST_1(nvm_file &f1, nvm_file& f2) {
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
    std::cout << svd.singularValues() << "\n";
    r = svd.matrixU() * svd.matrixV().transpose();
    if (r.determinant() < 0) {
      std::cerr << "Determinant obtained < 0\n";
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

  // scale = 16.0;
  // t << 0.137, -0.145, 0.86;

  // r << 0.6431,         0,   -0.7658,
  //     -0.1065,    0.9903,   -0.0895,
  //      0.7583,    0.1391,    0.6369;

  std::cout << "scale: " << scale << "\n";
  std::cout << "rotation " << r << "\n";
  std::cout << "trans " << t << "\n";
  struct RST rot_trans_scale;
  rot_trans_scale.scale = scale;
  rot_trans_scale.r = r;
  rot_trans_scale.t = t;
  return rot_trans_scale;
} 


// nvm_file merge_nvm_v2(nvm_file &f1, nvm_file &f2, struct RST rot_trans_scale){
//   float scale = rot_trans_scale.scale;
//   Eigen::MatrixXf r = rot_trans_scale.r;
//   Eigen::Vector3f t = rot_trans_scale.t;
  
//   for(int i =  0 ; i < f2.corr_data.size() ; i++){
//     f2.corr_data[i].point_3d = scale*r*f2.corr_data[i].point_3d + t;
//   }

//   for (int i=0; i<f2.kf_data.size(); i++) {
//     f2.kf_data[i].rotation = f2.kf_data[i].rotation * r.transpose();
//     f2.kf_data[i].translation = scale*f2.kf_data[i].translation - f2.kf_data[i].rotation*t;
//   }

//   nvm_file output;
//   assert(f1.description == f2.description);
//   output.description = f1.description;
//   for(int i = 0 ; i < f1.kf_data.size() ; i++){
//     output.kf_data.push_back(f1.kf_data[i]);
//   }
//   int maxsiftid = -1;
//   for(int i = 0 ; i < f1.corr_data.size() ; i++){
//     output.corr_data.push_back(f1.corr_data[i]);
//     if(maxsiftid > f1.corr_data[i].corr[0].siftid){
//       maxsiftid = f1.corr_data[i].corr[0].siftid;
//     }
//   }
//   assert(maxsiftid > 0);
//   maxsiftid++;
//   string sec_img = "img2_";
//   int num = 0;
//   unordered_map<string,int> f2_position;
//   for(int i = 0 ; i < f2.keyframe_data.size() ; i++){
//     output.keyframe_data.push_back(f2.keyframe_data[i]);
//     output.keyframe_data[output.keyframe_data.size() - 1].filename = sec_img + to_string(num) + ".jpg";
//     f2_position[f2.keyframe_data[i].filename] = output.size() - 1;
//     num++;
//   }

//   for(int i = 0 ; i < f2.corr_data.size() ; i++){
//     for(int j = 0 ; j < f2.corr_data[i].corr.size() ; j++){
//       struct imgcorr new_point = f2.corr_data[i].corr[j];
//       new_point.imgid = f2_position[f2.keyframe_data[new_point.imgid].filename];
//       new_point.siftid = maxsiftid;
//     }
//     maxsiftid++;
//   }
//   return output;
// }



nvm_file merge_nvm_1(nvm_file &f1, nvm_file &f2, struct RST rot_trans_scale) {
  float scale = rot_trans_scale.scale;
  Eigen::MatrixXf r = rot_trans_scale.r;
  Eigen::Vector3f t = rot_trans_scale.t;
  
  for(int i =  0 ; i < f2.corr_data.size() ; i++){
  	f2.corr_data[i].point_3d = scale*r*f2.corr_data[i].point_3d + t;
  }

  for (int i=0; i<f2.kf_data.size(); i++) {
    f2.kf_data[i].rotation = f2.kf_data[i].rotation * r.transpose();
    f2.kf_data[i].translation = scale*f2.kf_data[i].translation - f2.kf_data[i].rotation*t;
  }

  nvm_file output;
  assert(f1.description == f2.description);
  output.description = f1.description;
  for (int i=0; i<f1.kf_data.size(); i++) {
    output.kf_data.push_back(f1.kf_data[i]);
  }
  // for(int i =0  ; i < f2.kf_data.size() ; i++){
  //   output.kf_data.push_back(f2.kf_data[i]);
  // }
  for(int i = 0 ; i < f1.corr_data.size() ; i++){
    output.corr_data.push_back(f1.corr_data[i]);
  }
  // for(int i = 0 ; i < f2.corr_data.size() ; i++){
  //   output.corr_data.push_back(f2.corr_data[i]);
  // }

  // int maxsiftid = -1;
  // for(int i = 0 ; i < f1.corr_data.size() ; i++){
  //   output.corr_data.push_back(f1.corr_data[i]);
  //   if(maxsiftid < f1.corr_data[i].corr[0].siftid){
  //     maxsiftid = f1.corr_data[i].corr[0].siftid;
  //   }
  // }

  // assert(maxsiftid > 0);
  // maxsiftid++;

  // for(int i = 0 ; i < f2.corr_data.size() ; i++){
  //   for(int j = 0 ; j < f2.corr_data[i].size() ; j++){
  //     f2.corr_data.push_back()
  //   }
  // }
  
  // int offset = 0;
  // if (f1.kf_data.rbegin()->filename == f2.kf_data.begin()->filename) {
  //   offset = 1;
  // }
  
  // for (int i=offset; i<f2.kf_data.size(); i++) {
  //   output.kf_data.push_back(f2.kf_data[i]);
  // }

  // std::unordered_map<int, Corr3D> all_mappings;
  // for (int i=0; i<f1.corr_data.size(); i++) {
  //   all_mappings[f1.corr_data[i].corr[0].siftid] = f1.corr_data[i];
  // }
  // offset = f1.kf_data.size()-offset;

  // for (int i=0; i<f2.corr_data.size(); i++) {
  //   int sftid = f2.corr_data[i].corr[0].siftid;
  //   if (all_mappings.find(sftid) == all_mappings.end()) {
  //       all_mappings[sftid].point_3d = f2.corr_data[i].point_3d;
  //       all_mappings[sftid].color = f2.corr_data[i].color;  
  //     for (int j=0; j<f2.corr_data[i].corr.size(); j++) {
  //       all_mappings[sftid].corr.push_back(fix_corr(f2.corr_data[i].corr[j], offset));
  //     }
  //   } else {
  //     // sift id existed
  //     for (int j=0; j<f2.corr_data[i].corr.size(); j++) {
  //       if (f2.corr_data[i].corr[j].imgid>0) {
  //         all_mappings[sftid].corr.push_back(fix_corr(f2.corr_data[i].corr[j], offset));
  //       } 
  //     }
  //   }
  // }

  // for (auto it: all_mappings) {
  //   output.corr_data.push_back(it.second);
  // }  
  // std::cout << "Number of points " << output.corr_data.size() << "\n";

  return output;
}

void GetMatch(string folder1, string folder2, string nvm_file1, string nvm_file2, float cx, float cy){

	struct nvm_file file1_kf(nvm_file1);
	struct nvm_file file2_kf(nvm_file2);

  cout << "nvm file read\n";

	vector<keyframe_data> key_frames_file1 = file1_kf.kf_data;
	vector<keyframe_data> key_frames_file2 = file2_kf.kf_data;

	for(int i = 0 ; i < key_frames_file1.size() ; i++){
		first_file[key_frames_file1[i].filename] = key_frames_file1[i];
	}

	for(int i = 0 ; i < key_frames_file2.size() ; i++){
		second_file[key_frames_file2[i].filename] = key_frames_file2[i];
	}

	for(int i = 0 ; i < key_frames_file1.size() ; i++){
		position_first[key_frames_file1[i].filename] = i;
	}

	for(int i = 0 ; i < key_frames_file2.size() ; i++){
		position_second[key_frames_file2[i].filename] = i;
	}

	for(int i = 0 ; i < key_frames_file1.size() ; i += key_frames_file1.size()/5){
		for(int j = 0; j < key_frames_file2.size() ; j += key_frames_file2.size()/5){
			string file1 = folder1 + '/' + key_frames_file1[i].filename;
			string file2 = folder2 + '/' + key_frames_file2[j].filename;
			cout << "running sift for :  " << key_frames_file1[i].filename << "  " << key_frames_file2[j].filename << "\n";
      struct sift_corr corres = Running_SIFT(file1,file2);
      cout << "sift run completed for :  " << i << "  " << j << "\n";
			for(int k = 0 ; k < corres.first_img.size() ; k++){
				struct Point_Frame new_point1;
        // cout << k << endl;
        
        new_point1.x = corres.first_img[k].x;
        new_point1.y = corres.first_img[k].y;
        new_point1.frame_id = key_frames_file1[i].filename;
        
        
        struct Point_Frame new_point2;
        new_point2.x = corres.second_img[k].x;
        new_point2.y = corres.second_img[k].y;
        new_point2.frame_id = key_frames_file2[j].filename;

        cout<<new_point1.x<<" "<<new_point1.y<<" "<<new_point2.x<<" "<<new_point2.y<<endl;

				if(point_to_siftid_img1.count(new_point1) > 0 && point_to_siftid_img2.count(new_point2) > 0){
          // cout<<point_to_siftid_img1[new_point1]<<" "<<point_to_siftid_img2[new_point2]<<"#####\n";
          // assert (point_to_siftid_img1[new_point1]==point_to_siftid_img2[new_point2]);
					//donot do anything
				}
				else if(point_to_siftid_img1.count(new_point1) > 0 && point_to_siftid_img2.count(new_point2) == 0){
					int sid = point_to_siftid_img1[new_point1];
					point_to_siftid_img2[new_point2] = sid;
					siftid_to_point_img2[sid].push_back(new_point2);
				}
				else if(point_to_siftid_img2.count(new_point2) > 0 && point_to_siftid_img1.count(new_point1) == 0){
					int sid = point_to_siftid_img2[new_point2];
					point_to_siftid_img1[new_point1] = sid;
					siftid_to_point_img1[sid].push_back(new_point1);
				}
				else{
					point_to_siftid_img1[new_point1] = siftid;
					point_to_siftid_img2[new_point2] = siftid;
					siftid_to_point_img1[siftid].push_back(new_point1);
					siftid_to_point_img2[siftid].push_back(new_point2);
					siftid++;
				}
        // cout << "doing loop "<< k << endl;
			}
      cout << "matching completed file  : " <<i << "  " << j << endl;
		}
	}

	vector<struct Corr3D> all3D_Points_img1;
	vector<struct Corr3D> all3D_Points_img2;

	struct nvm_file first_img_nvm;
	struct nvm_file second_img_nvm;


  bool take = true;
  int given_sift;
	for(auto it = siftid_to_point_img1.begin() ; it != siftid_to_point_img1.end() ; it++){
		vector<Point_Frame> all_images;
		all_images = it -> second;
    if(all_images.size()<=1)
      continue;

		vector<triangulation_bundle> all_points;
		vector<imgcorr> corres_images1;
		for(int k = 0 ; k < all_images.size() ; k++){
			Point_Frame given_point = all_images[k];
      cout << folder1 + given_point.frame_id << endl;
      cv::Mat im1 = cv::imread(folder1 + given_point.frame_id,  CV_LOAD_IMAGE_COLOR);
      imwrite(output_path + given_point.frame_id,im1);

      // if(take){
      //   cout << "ghus gaya mc\n";
      //   cout << folder1 + given_point.frame_id << endl;
      //   cv::Mat im1 = cv::imread(folder1 + given_point.frame_id);
      //   cv::circle(im1,Point(given_point.x,given_point.y), 7.5, cv::Scalar(255,0,0), 1, 8, 0);
      //   imwrite(output_path + given_point.frame_id,im1);
      // }
			keyframe_data corres_frame = first_file[given_point.frame_id];
			camera_frame_wo_image param((float)corres_frame.focal,corres_frame.rotation, corres_frame.translation,cx,cy);
			cv::Point2f pt(given_point.x - cx, given_point.y - cy);
			triangulation_bundle new_point = triangulation_bundle(param,pt);
			all_points.push_back(new_point);
			imgcorr new_corr;
			
			new_corr.imgid = position_first[given_point.frame_id];
			new_corr.siftid = it -> first;
			new_corr.img_location = pt;
			corres_images1.push_back(new_corr);

		}
    take = false;
    cout<<"All points size:"<<all_points.size()<<endl;
		cv::Point3f new3D_point = Triangulate(all_points);
		cv::Point3i color;
		color.x = 255;
		color.y = 255;
		color.z = 255;
		Corr3D new3D(new3D_point,color);
		new3D.corr = corres_images1;
		all3D_Points_img1.push_back(new3D);
	}

	first_img_nvm.kf_data = key_frames_file1;
	first_img_nvm.corr_data = all3D_Points_img1;
	first_img_nvm.description = file1_kf.description;
	first_img_nvm.median_val = file1_kf.median_val;

	for(auto it = siftid_to_point_img2.begin() ; it != siftid_to_point_img2.end() ; it++){
		vector<Point_Frame> all_images;
		all_images = it -> second;
    if(all_images.size()<=1)
      continue;
		vector<triangulation_bundle> all_points;
		vector<imgcorr> corres_images2;
		for(int k = 0 ; k < all_images.size() ; k++){
			Point_Frame given_point = all_images[k];
			keyframe_data corres_frame = second_file[given_point.frame_id];
			camera_frame_wo_image param((float)corres_frame.focal,corres_frame.rotation, corres_frame.translation,cx,cy);
			cv::Point2f pt(given_point.x - cx, given_point.y - cy);
			triangulation_bundle new_point = triangulation_bundle(param,pt);
			all_points.push_back(new_point);
			imgcorr new_corr;

			new_corr.imgid = position_second[given_point.frame_id];
			new_corr.siftid = it -> first;
			new_corr.img_location = pt;
			corres_images2.push_back(new_corr);
		}

    cout<<"All points size:"<<all_points.size()<<endl;
		cv::Point3f new3D_point = Triangulate(all_points);
		cv::Point3i color;
		color.x = 255;
		color.y = 255;
		color.z = 255;
		Corr3D new3D(new3D_point,color);
		new3D.corr = corres_images2;
		all3D_Points_img2.push_back(new3D);
	}

	second_img_nvm.kf_data = key_frames_file2;
	second_img_nvm.corr_data = all3D_Points_img2;
	second_img_nvm.description = file1_kf.description;
	second_img_nvm.median_val = file1_kf.median_val;

	struct RST rot_trans_scale =  GetBestRST_1(first_img_nvm,second_img_nvm);
	struct nvm_file final_output = merge_nvm_1(file1_kf,file2_kf,rot_trans_scale);
	string output_dir = "/home/vision/Desktop/btp_data/";
	final_output.save_to_disk(output_dir + "combined.nvm");
	final_output.save_ply_file(output_dir + "combined.ply");
}


int main(int argc, char **argv){
  string nvm_file1 ="/home/vision/Desktop/btp_data/data0/outputVSFM_GB.nvm" ;
  string nvm_file2 ="/home/vision/Desktop/btp_data/data1/outputVSFM_GB.nvm" ;
  string folder1 = "/home/vision/Desktop/btp_data/data0/";
  string folder2 = "/home/vision/Desktop/btp_data/data1/";
  int cx = 640;
  int cy = 273; 
  GetMatch(folder1, folder2, nvm_file1, nvm_file2, cx, cy);

  return 0;
}