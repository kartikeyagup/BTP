#include "utils.h"

bool isPerformIncrementalBA = false;      /**< Flag indicating whether to perform incremental BA or not. */
bool useRansacCameraEstimate;     /**< Flag indicating whether or not to use ransac camera estimates. */
int nCameras;              /**< Number of valid cameras. Could change dynamically. */
int nActualCameras;        /**< Actual number of views in dataset. */
int nWorkingCameras;       /**< Number of views with which we are working in current iteration. */
int newCameraIndex = -1;   /**< Index of the newly selected camera. Set only in incr BA. */

bool isSaveKeySeqToDisk = false;
bool isLoadKeySeqFromDisk = false;
bool isSavePairWiseCorrsToDisk = false;
bool isLoadPairWiseCorrsToDisk = false;

KeyPointType keyPtType;
string strImagePrefix;
string strImageExtn;
string strKeyPrefix;
string strKeyExtn;
int nDigits;
int nStartIndex;
int nEndIndex;

string strPairwiseCorrsDir;
bool useHomographyRansacer = true;
double ransacThreshold = 1.0;

// Variables for maintaining world points and their projections

//swati code starts
//int kinect_index;
//bool *visibleInKinect = NULL;
//swati code starts

int nWorldPts;                                 /**< Number of current iteration's world points.  */
int nOrigWorldPts;                             /**< Number of original world points.  */
int nFeatureCorrs;                             /**< Maximum number of feature correspondences for a single world point. */
string strWorldPtsProjectionFile = "worldPtProjections.txt";

bool useCurrentEstimates = false;              /**< Flag indicating whether current estimates will be used or not. */
bool useStableEstimates = false;               /**< Flag indicating whether stable estimates will be used or not. */
bool *pIsValidCamera = NULL;                   /**< Array maintaining which cameras are valid and which are not. */
bool *pIsValidWorldPt = NULL;                  /**< Array maintaining which world points are valid and which are not. */
bool *pIsPrevIterValidCamera = NULL;           /**< Array maintaining which cameras are valid and which are not. */
bool *pIsPrevIterValidWorldPt = NULL;          /**< Array maintaining which world points are valid and which are not. */
bool *pIsCurIterValidCamera = NULL;            /**< Array maintaining which cameras are valid in current iteration. */
bool *pIsCurIterValidWorldPt = NULL;           /**< Array maintaining which world points are valid in current iteration. */
bool *pIsInvalidatedCamera = NULL;             /**< Array maintaining which cameras have been invalidated. */
bool *pIsInvalidatedWorldPt = NULL;            /**< Array maintaining which world points have been invalidated. */

int nProjPtParams;
int nValidWorldPtProjs;
int nPositionParams, nOrientationParams, nIntrinsicParams;
int nSingleCamParams, nWorldPtParams;
bool amCalibratingStructure, amCalibratingCameraIntrinsics, amCalibratingCameraOrientations, amCalibratingCameraPositions;
char *pProjVisibilityMask = NULL;       /**< Binary array indicating whether pt i is visible in image j or not. */
double *pWorldPtProjections;     /**< Array containing all valid world point projections. */
double *pInitialParamEstimate;   /**< Initial parameter vector consisting of camera parameters and projection pts. */
double *global_last_ws;
double *global_last_Rs;

double curErrorThresh = 1.0;
bool isFirstBAIteration = true;
double *pCurrentCameraEstimate = NULL;    /**< Current camera parameter vector. */
double *pCurrentWorldPtEstimate = NULL;   /**< Current world point parameter vector. */
double *pStableCameraEstimate = NULL;     /**< Stable camera parameter vector. */
double *pStableWorldPtEstimate = NULL;    /**< Stable world point parameter vector. */


int verbosityLevel;                   /**< Verbosity level of sparse bundle adjustment. */
double *pMinimizationOptions;         /**< Options configured for performing sparse bundle adjustment based minimization. */
double *pOutputInfoOptions;           /**< After performing sparse bundle adjustment, contains info regarding outcome of minimization. */

int nMaxViews;
int maxViewsFlag;

