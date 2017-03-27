#ifndef _CORRESPONDENCE_GENERATOR_H_
#define _CORRESPONDENCE_GENERATOR_H_

#include <vector>


#include "utils.h"
#include "RobHessSiftKeyPointDetector.h"

using namespace std;

class KeyPointWithDesc
{
  public:

    // Default constructor
    KeyPointWithDesc ()
    {
      m_d = NULL;
      m_x = m_y = 0.0;
      m_extra = m_track = -1;

      m_keyPtType = UNDEFINED_KEY_POINT;
      m_scale = m_orient = -1;
      m_laplacian = m_size = m_hessian = -1;
    }

    // Parameterized constructor : initializes key point location and descriptor
    KeyPointWithDesc (double x, double y, double *d, KeyPointType keyPtType = UNDEFINED_KEY_POINT) : m_d(d)
    {
      m_x = x; m_y = y;
      m_extra = -1; m_track = -1;
      m_keyPtType = keyPtType;
    }

    //// Destructor
    //virtual ~KeyPointWithDesc ();

    double* GetDesc () { return m_d; }

    double m_x, m_y;                      /* Subpixel location of key point. */
    unsigned char m_r, m_g, m_b;          /* Color of this key */
    int m_extra;                          /* 4 bytes of extra storage */
    int m_track;                          /* Track index this point corresponds to */
    KeyPointType m_keyPtType;             /** Type of this keypoint. */

    double *m_d;                            /**< Vector of descriptor values */
    int m_nDescriptors;                     /**< Size of descriptor array. */
    double m_scale;                         /**< Scale at which this point is a SIFT feature. */
    double m_orient;                        /**< Orientation of the feature. */
    int m_laplacian;                        /**< -1, 0 or +1. sign of the laplacian at the point. */
    int m_size;                             /**< Size of the feature. */
    float m_hessian;                        /**< Value of the hessian (can approx. estimate feature strengths). */
};

class KeyPointMatch
{
  public:

    // Default constructor
    KeyPointMatch () {}

    // Parameterized constructor : Initializes the matching indices
    KeyPointMatch (int idx1, int idx2) : m_idx1 (idx1), m_idx2 (idx2)
    {}

    int m_idx1, m_idx2;
};

typedef struct CvSIFTParams
{
  int m_intervals;
  double m_sigma;
  double m_contThreshold;
  int m_curvatureThreshold;
  int m_imageDoublingPrior;
  int m_descWidth;
  int m_descHistogramBins;

  CvSIFTParams ()
  {
    m_intervals= SIFT_INTVLS;
    m_sigma = SIFT_SIGMA;
    m_contThreshold = SIFT_CONTR_THR;
    m_curvatureThreshold = SIFT_CURV_THR;
    m_imageDoublingPrior = SIFT_IMG_DBL;
    m_descWidth = SIFT_DESCR_WIDTH;
    m_descHistogramBins = SIFT_DESCR_HIST_BINS;
  }
} CvSIFTParams;

typedef vector<KeyPointWithDesc> VecOfKeyPointWithDesc;
typedef vector<VecOfKeyPointWithDesc> TableOfKeyPointWithDesc;
typedef vector<KeyPointMatch> VecOfKeyPointMatch;

extern TableOfKeyPointWithDesc keyPointTable;      /**< Structure maintaining key points for each image. */
//extern VecOfKeyPointMatch *pPairwiseKeyPointMatches;         /**< Structure maintaining pairwise key point matches. */
//extern VecOfFeatureList corrsTable;
extern vector<vector<pair<pair<int,int>,pair<double,double >> > > corrsTable;
extern  vector<vector<pair<int,pair<int,int>>>>image_to_track;;
	// Detects key points for an image sequence
void detectKeyPointsInImageSeq ();

// Computes pair wise correspondences for an image sequence
void computePairWiseCorrs ();

// Computes N-View correspondences for an image sequence
//void computeNViewCorrs ();
void computeNViewCorrs (int nViews);
// Clears the key point table
void clearKeyPointTable ();
void corresGenerate(int numImage,vector<int>&mapping, char*filename);
// Clears the in-memory pair wise matches storage
void clearPairwiseMatches ();
void saveNViewCorrsToDisk (int nViews,char*outfilename, char*outfilename2, string path, vector <string> listfile);
// Clears the correspondence table
void clearCorrsTable ();

// Detects key points on the input image
int detectSiftKeyPoints (IplImage* image, VecOfKeyPointWithDesc& keyPtDescVec);
int detectSurfKeyPoints (IplImage* image, VecOfKeyPointWithDesc& keyPtDescVec);



VecOfKeyPointMatch removeDuplicateMatches (const VecOfKeyPointMatch &matches,
                                           VecOfKeyPointWithDesc& keyPointList1,
                                           VecOfKeyPointWithDesc& keyPointList2);

// Retrieves the list of good matches
int getListOfGoodMatches (const int curPairIndex, 
                          const int imageIdx1, const int imageIdx2, 
                          int& nGoodMatches, bool **goodMatchIndexArr);

void loadKeyPointSeqFromDisk ();

// Loads a correspondence table from disk
void loadCorrespondenceTable (const char *strFilename);

void saveNViewCorrsToDisk ();

// Initialize internal structures from in-memory correspondence table structure.
void loadFromCorrsTable ();

void loadPairwiseCorrsFromDisk ();
void savePairwiseCorrsToDisk (const int imageIdx1, const int imageIdx2, const VecOfKeyPointMatch& keyPointMatchesVec);

VecOfKeyPointWithDesc loadSiftKeyFromFile (const char *filename);
VecOfKeyPointWithDesc loadSurfKeyFromFile (const char *filename);

int saveSiftKeyToFile (const char *filename, VecOfKeyPointWithDesc keyPtDescVec);
int saveSurfKeyToFile (const char *filename, VecOfKeyPointWithDesc keyPtDescVec);

#endif // _CORRESPONDENCE_GENERATOR_H_
