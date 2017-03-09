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

#include "ceres_fit.h"
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

void optimize(plane& rf, plane& lft, plane& rt, std::vector<cv::Point3f> pts_rf,
              std::vector<cv::Point3f> pts_left,
              std::vector<cv::Point3f> pts_right) {
  // google::InitGoogleLogging(argv[0]);
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
  // plyfile f_wall1("test/Wall1.ply");
  // plyfile f_wall2("test/Wall2.ply");
  // plyfile f_roof("test/Roof.ply");

  // std::cout << "Read input\n";
  for (int i = 0; i < pts_left.size(); ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(
            new PlanarResidual((double)pts_left[i].x, (double)pts_left[i].y,
                               (double)pts_left[i].z)),
        NULL, &a, &b, &c, &d);
  }
  for (int i = 0; i < pts_right.size(); ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(
            new PlanarResidual((double)pts_right[i].x, (double)pts_right[i].y,
                               (double)pts_right[i].z)),
        NULL, &a, &b, &c, &e);
  }
  for (int i = 0; i < pts_rf.size(); ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<PlanarResidual, 1, 1, 1, 1, 1>(
            new PlanarResidual((double)pts_rf[i].x, (double)pts_rf[i].y,
                               (double)pts_rf[i].z)),
        NULL, &k, &l, &m, &n);
  }
  problem.AddResidualBlock(
      new AutoDiffCostFunction<PerpResidual, 1, 1, 1, 1, 1, 1, 1>(
          new PerpResidual()),
      NULL, &a, &b, &c, &k, &l, &m);

  problem.SetParameterBlockConstant(&d);
  problem.SetParameterBlockConstant(&n);
  Solver::Options options;
  options.max_num_iterations = 1000;
  options.linear_solver_type = ceres::DENSE_QR;
  options.use_nonmonotonic_steps = true;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << "\n";
  std::cout << "Final 1 " << a << "\t" << b << "\t" << c << "\t" << d << "\t"
            << e << "\n";
  std::cout << "Final 2 " << k << "\t" << l << "\t" << m << "\t" << n << "\n";
  std::cout << "Error: "
            << (a * k + b * l + c * m) /
                   sqrt((a * a + b * b + c * c) * (k * k + l * l + m * m))
            << "\n";
  rf.a = k;
  rf.b = l;
  rf.c = m;
  rf.d = -n;
  rf.normalize();
  lft.a = a;
  lft.b = b;
  lft.c = c;
  lft.d = -d;
  lft.normalize();
  rt.a = a;
  rt.b = b;
  rt.c = c;
  rt.d = -e;
  rt.normalize();

  // return 0;
}
