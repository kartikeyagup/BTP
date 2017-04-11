#ifndef HELPERS_H
#define HELPERS_H

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "camera_intrinsics_prior.h"
#include "correspondance.h"
#include "estimate_twoview_info.h"
#include "feature_correspondence.h"
#include "twoview_info.h"
#include "verify_two_view_matches.h"

bool GetEssentialRT(corr& corres, TwoViewInfo& twoview_info,
                    std::vector<int>& inlier_indices,
                    VerifyTwoViewMatchesOptions& options, float focal);

bool VerifyTwoViewMatches(
    const VerifyTwoViewMatchesOptions& options,
    const CameraIntrinsicsPrior& intrinsics1,
    const CameraIntrinsicsPrior& intrinsics2,
    const std::vector<FeatureCorrespondence>& correspondences,
    TwoViewInfo* twoview_info, std::vector<int>* inlier_indices);

void ShowCorres(std::string FLAGS_dirname, corr compressed);

#endif
