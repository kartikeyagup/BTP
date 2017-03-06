#include <cstdio>
#include <vector>
#include "ceres/ceres.h"
using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

float distance(float a, float b, float c, float d, float x, float y, float z) {
  float num = a*x + b*y + c*z + d;
  float den = sqrt(a*a + b*b + c*c);
  return fabs(num/den);
}

class DistanceFromPlane {
  private:
    float x, y, z;
    int type;
  public:
    DistanceFromPlane(Point3f inp, int t) : x(inp.x), y(inp.y), z(inp.z), type(t) {}
    bool operator()(const float* const a1, const float* const b1,
                    const float* const c1, const float* const d1,
                    const float* const d2, const float* const a3,
                    const float* const b3, const float* const c3,
                    const float* const d3, const float* residual) const {
      //a1, b1, c1 and d1 parameters of the first plane equation is 
      //given by a1x + b1y + c1z + d1 = 0
      //a1, b1, c1 and d2 parameter of parallel plane
      //a3, b3, c3 and d3 parameter of perpendicular plane
      if(type == 1) {
        residual[0] = distance(*a1, *b1, *c1, *d1, x, y, z);
      }
      else if(type == 2) {
        residual[0] = distance(*a1, *b1, *c1, *d2, x, y, z);
      }
      else if(type == 3) {
        residual[0] = distance(*a3, *b3, *c3, *d3, x, y, z);
      }
      else {
        residual[0] = (*a1)*(*a3) + (*b1)*(*b3) + (*c1)*(*c3);
      }
      return true;
    }

};


int main(int argc, char** argv) {
  //reading ply file
  //first file is for the one plane second for the parallel plane and third for the perpendicular plane
  string file1 = "";
  ply_file points1(file1);
  string file2 = "":
  ply_file points2(file2);
  string file3 = "";
  ply_file points3(file3);

  //build the problem
  Problem problem;

  //Configure the loss function.
  LossFunction* loss = NULL;
  if (FLAGS_robust_threshold) {
    loss = new CauchyLoss(0.0);
  }

  //variables 
  float a1, b1, c1, d1, d2, a3, b3, c3, d3;
  a1 = 1.0; b1 = 1.0; c1 = 1.0; d1 = 1.0; d2 = 1.0;
  a3 = 1.0; b3 = 1.0; c3 = 1.0; d3 = 1.0; 

  //Add the residuals.
  int num_points = 0;
  for(int i = 0 ; i < points1.point_data.size() ; i++) {
    CostFunction *cost =
      new AutoDiffCostFunction<DistanceFromPlane, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
            new DistanceFromPlane(points1.point_data[i].point,1));
    problem.AddResidualBlock(cost, loss, &a1, &b1, &c1, &d1, &d2, &a3, &b3, &c3, &d3);
    num_points++;
  }

  for(int i = 0 ; i < points2.point_data.size() ; i++) {
    CostFunction *cost =
      new AutoDiffCostFunction<DistanceFromPlane, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
            new DistanceFromPlane(points2.point_data[i].point,1));
    problem.AddResidualBlock(cost, loss, &a1, &b1, &c1, &d1, &d2, &a3, &b3, &c3, &d3);
    num_points++;
  }

  for(int i = 0 ; i < points3.point_data.size() ; i++) {
    CostFunction *cost =
      new AutoDiffCostFunction<DistanceFromPlane, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
            new DistanceFromPlane(points3.point_data[i].point,1));
    problem.AddResidualBlock(cost, loss, &a1, &b1, &c1, &d1, &d2, &a3, &b3, &c3, &d3);
    num_points++;
  }
  std::cout << "Got " << num_points << " points.\n";
  
  // Build and solve the problem.
  Solver::Options options;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << a1 << "  " << b1 << "  " << c1 << "  " << d1 << "  " << d2 << "  " << "\n";
  std::cout << a3 << "  " << b3 << "  " << c3 << "  " << d3 << "\n";
  return 0;
}