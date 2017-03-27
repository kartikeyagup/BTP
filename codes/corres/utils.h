#ifndef _UTILS_H_
#define _UTILS_H_

#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2\opencv.hpp>


using namespace std;

// Enumerator for type of key points (which in turn will define key points descriptor)
enum KeyPointType {UNDEFINED_KEY_POINT = -1, SIFT_KEY_POINT, SURF_KEY_POINT};

#define MAXITER 150
#define WIDTH (972)
#define HEIGHT (648)
#define CLAMP(x,mn,mx) (((x) < mn) ? mn : (((x) > mx) ? mx : (x)))

// Macros to be used for deleting objects or array of objects
#define SAFE_DELETE_PTR(p) {if (p) {delete (p); (p) = NULL;}}
#define SAFE_DELETE_ARR(p) {if (p) {delete[] (p); (p) = NULL;}}

// Type definitions
typedef CvPoint2D64f Feature;
typedef CvPoint2D64f* FeatureList;
typedef vector<FeatureList> VecOfFeatureList;
typedef CvPoint3D64f Feature3;
typedef CvPoint3D64f* FeatureList3;
typedef vector<Feature3> Feature3Vec;

void printEstimatedParams ();
void dumpAllStructures (int nIterCount);
void loadMatrix (CvMat **mat, FILE *fpIn);
void printMatrix (FILE *fpOut, CvMat *mat);
void normalizeMatrix (CvMat *mat);
void computeVisionCameraMatrix ();
int constructArrStructures ();
void computeProjEstimate (double *outProjEstimate);

double median_copy(int n, double *arr);
double findKthElement(int n, int k, double *arr);
double kth_element_copy(int n, int k, double *arr);
int iround(double x);
static int partition(int n, double *arr);

// Entry point for performing key point detection, matching etc.
int main1 (int argc, char **argv);
extern int colors[3];
extern bool isPerformIncrementalBA;      /**< Flag indicating whether to perform incremental BA or not. */
extern bool useRansacCameraEstimate;     /**< Flag indicating whether or not to use ransac camera estimates. */
extern int nCameras;              /**< Number of valid cameras. Could change dynamically. */
extern int nActualCameras;        /**< Actual number of views in dataset. */
extern int nWorkingCameras;       /**< Number of views with which we are working in current iteration. */
extern int newCameraIndex;        /**< Index of the newly selected camera. Set only in incr BA. */

extern int END_INDEX;
extern bool isSaveKeySeqToDisk;
extern bool isLoadKeySeqFromDisk;
extern bool isSavePairWiseCorrsToDisk;
extern bool isLoadPairWiseCorrsToDisk;

extern KeyPointType keyPtType;
extern string strImagePrefix;
extern string strImageExtn;
extern string strKeyPrefix;
extern string strKeyExtn;
extern int nDigits;
extern int nStartIndex;
extern int nEndIndex;

extern string strPairwiseCorrsDir;
extern bool useHomographyRansacer;
extern double ransacThreshold;

// Variables for maintaining world points and their projections
extern int nWorldPts;                                 /**< Number of current iteration's world points.  */
extern int nOrigWorldPts;                             /**< Number of original world points.  */
extern int nFeatureCorrs;                             /**< Maximum number of feature correspondences for a single world point. */
extern string strWorldPtsProjectionFile;
extern Feature3Vec vecOfWorldPts;                     /**< Current copy of the world points. */
extern vector<Feature3Vec> vecOfWorldPtProjs;         /**< Current copy of the world point projections. */
//extern Feature3Vec vecOfOrigWorldPts;                 /**< Original copy of the world points. */
//extern vector<Feature3Vec> vecOfOrigWorldPtProjs;     /**< Original copy of the world point projections. */

extern bool useCurrentEstimates;                         /**< Flag indicating whether current estimates will be used or not. */
extern bool useStableEstimates;                          /**< Flag indicating whether stable estimates will be used or not. */
extern bool *pIsValidCamera;                             /**< Array maintaining which cameras are valid and which are not. */
extern bool *pIsValidWorldPt;                            /**< Array maintaining which world points are valid and which are not. */
extern bool *pIsPrevIterValidCamera;                     /**< Array maintaining which cameras were valid in previous iteration. */
extern bool *pIsPrevIterValidWorldPt;                    /**< Array maintaining which world points were valid in previous iteration. */
extern bool *pIsCurIterValidCamera;                      /**< Array maintaining which cameras are valid in current iteration. */
extern bool *pIsCurIterValidWorldPt;                     /**< Array maintaining which world points are valid in current iteration. */
extern bool *pIsInvalidatedCamera;                       /**< Array maintaining which cameras have been invalidated. */
extern bool *pIsInvalidatedWorldPt;                      /**< Array maintaining which world points have been invalidated. */

extern int nProjPtParams;
extern int nValidWorldPtProjs;
extern int nPositionParams, nOrientationParams, nIntrinsicParams;
extern int nSingleCamParams, nWorldPtParams;
extern bool amCalibratingStructure, amCalibratingCameraIntrinsics, amCalibratingCameraOrientations, amCalibratingCameraPositions;
extern FeatureList3 worldPtsArr;        /**< Array containing the world point estimates. */
extern FeatureList* featureCorrsArr;    /**< Array of feature correspondence arrays. */
extern char *pProjVisibilityMask;       /**< Binary array indicating whether pt i is visible in image j or not. */
extern double *pWorldPtProjections;     /**< Array containing all valid world point projections. */
extern double *pInitialParamEstimate;   /**< Initial parameter vector consisting of camera parameters and projection pts. */

// Utility matrices
extern CvMat *tmpMat;
extern CvMat *nx;
extern CvMat *nxsq;
extern CvMat *nxT;
extern CvMat *nxsqT;
extern CvMat *identityMat;
extern CvMat *intrinsicMatrix;
extern CvMat *rotMatrix;
extern CvMat *translationVec;
extern CvMat *visionCamera;
extern CvMat *visionCamera2;
extern CvMat *worldPtMat;
extern CvMat *projPtMat;
extern CvMat *lastProjPtMat;
extern CvMat *tmpTrans;
extern IplImage **image;
extern int verbosityLevel;                   /**< Verbosity level of sparse bundle adjustment. */
extern double *pMinimizationOptions;         /**< Options configured for performing sparse bundle adjustment based minimization. */
extern double *pOutputInfoOptions;           /**< After performing sparse bundle adjustment, contains info regarding outcome of minimization. */
extern double *global_last_ws;
extern double *global_last_Rs;

extern double curErrorThresh;
extern bool isFirstBAIteration;
extern double *pCurrentCameraEstimate;    /**< Current camera parameter vector. */
extern double *pCurrentWorldPtEstimate;   /**< Current world point parameter vector. */
extern double *pStableCameraEstimate;     /**< Stable camera parameter vector. */
extern double *pStableWorldPtEstimate;    /**< Stable world point parameter vector. */
extern int nMaxViews;
extern int maxViewsFlag;

#define CLAMP(x,mn,mx) (((x) < mn) ? mn : (((x) > mx) ? mx : (x)))

#endif // _UTILS_H_
