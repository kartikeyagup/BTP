// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "plyfile.h"
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct PlanarResidual {
  PlanarResidual(double x, double y, double z)
      : x_(x), y_(y), z_(z) {}
  template <typename T> bool operator()(const T* const a,
                                        const T* const b,
                                        const T* const c,
                                        const T* const d,
                                        T* residual) const {
    residual[0] = d[0] - (x_*a[0] + y_*b[0] + z_*c[0]);
    return true;
  }
 private:
  const double x_;
  const double y_;
  const double z_;
};

struct PerpResidual {
  template <typename T>
  bool operator()(const T* const x1,
                  const T* const x2,
                  const T* const x3,
                  const T* const x4,
                  const T* const x5,
                  const T* const x6,
                  T* residual) const {
    residual[0] = 100.0*(x1[0]*x4[0] + x2[0]*x5[0] + x3[0]*x6[0]);
    return true;
  }
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  double a = 1.0;
  double b = 1.0;
  double c = 1.0;
  double d = 1.0;
  double e = 1.0;
  double k = 1.0;
  double l = 1.0;
  double m = 1.0;
  double n = 1.0;
  Problem problem;
  plyfile f_wall1("test/Wall1.ply");
  plyfile f_wall2("test/Wall2.ply");
  plyfile f_roof("test/Roof.ply");

  std::cout << "Read input\n";
  for (int i = 0; i < f_wall1.NumPoints(); ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(
            new PlanarResidual(f_wall1.alldata[i].first.x,
                               f_wall1.alldata[i].first.y, 
                               f_wall1.alldata[i].first.z)),
        NULL,
        &a, &b, &c, &d);
  }
  for (int i = 0; i < f_wall2.NumPoints(); ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(
            new PlanarResidual(f_wall2.alldata[i].first.x,
                               f_wall2.alldata[i].first.y, 
                               f_wall2.alldata[i].first.z)),
        NULL,
        &a, &b, &c, &e);
  }
  for (int i = 0; i < f_roof.NumPoints(); ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(            
          new PlanarResidual(f_roof.alldata[i].first.x,
                             f_roof.alldata[i].first.y, 
                             f_roof.alldata[i].first.z)),
        NULL,
        &k, &l, &m, &n);
  }
  problem.AddResidualBlock(new AutoDiffCostFunction<PerpResidual, 1, 1, 1, 1, 1, 1, 1>(new PerpResidual()),
    NULL,
    &a, &b, &c, &k, &l, &m);

  problem.SetParameterBlockConstant(&d);
  problem.SetParameterBlockConstant(&n);
  Solver::Options options;
  options.max_num_iterations = 1000;
  options.linear_solver_type = ceres::DENSE_QR;
  options.use_nonmonotonic_steps = true;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  std::cout << "Final 1 " << a << "\t" << b << "\t" << c << "\t" << d << "\t" << e << "\n";
  std::cout << "Final 2 " << k << "\t" << l << "\t" << m << "\t" << n << "\n";
  std::cout << "Error: " <<(a*k + b*l + c*m)/sqrt((a*a + b*b + c*c)*(k*k + l*l + m*m)) << "\n";
  return 0;
}
