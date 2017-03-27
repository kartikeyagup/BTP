#include <iostream>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include<iostream>
#include<fstream>
#include<istream>
#include <iomanip>
#include<vector>
#include<string>
#include <cmath>
#include<map>
#include <list>
#include "CorrespondenceGenerator.h"
#include "utils.h"
int detectASiftKeyPoints (char *filename, VecOfKeyPointWithDesc& keyPtDescVec);
using namespace std;

const int DETECT_KEYPOINTS = 0;
const int GEN_PAIRWISE_CORRS = 1;
const int GEN_NVIEW_CORRS = 2;

int execMode;

// Prints command line usage information. Also, parses command line parameters.
bool usage1 (int argc, char **argv)
{
  bool isCorrectUsage = true;

  if ((argc != 8) && (argc != 10) && (argc != 6))
  {
    isCorrectUsage = false;
    //printf("\n swati!!"); fflush(stdout); exit(0);
	}
  if (argc > 1)
  {
    execMode = atoi (argv[1]);
    if (((execMode == DETECT_KEYPOINTS) && (argc != 8))
        || ((execMode == GEN_PAIRWISE_CORRS) && (argc != 8))
        || ((execMode == GEN_NVIEW_CORRS) && (argc != 10)))
    isCorrectUsage = false;
  }

  if (!isCorrectUsage)
  {
    printf ("----------------------------------------------------------------------\n");
    printf ("Usage:\n");
    printf ("%s 0 <keyPtType> <imagePrefix> <imageExtn> <nDigits> <startIndex> <endIndex>\n\n", argv[0]);
    printf ("%s 1 <keyPtType> <keyPrefix> <keyExtn> <nDigits> <startIndex> <endIndex>\n\n", argv[0]);
    printf ("%s 2 <keyPtType> <keyPrefix> <keyExtn> <nDigits> <startIndex> <endIndex> <corrsFileDir> <fittingFunction>\n\n", argv[0]);

    printf ("\n");
    printf ("execMode : Mode of execution\n");
    printf ("         : 0 - Detect keypoints in an image sequence and dump them to disk.\n");
    printf ("         : 1 - Load key points and generate pair wise correspondences with duplicate matches removed.\n");
    printf ("         : 2 - Load keypoints / pairwise corrs to generate n-view corrs (world point projection file).\n");
    printf ("\n");

    printf ("keyPtType              : Key point type to be used (0 - SIFT; 1 - SURF).\n");	//swati take it as zero.
    printf ("imagePrefix            : Relative or absolute path name to the image files.\n");
    printf ("imageExtn              : Image file extensions.\n");
    printf ("keyPrefix              : Relative or absolute path name to the key files.\n");
    printf ("keyExtn                : Key file extensions.\n");
    printf ("nDigits                : Number of digits (including leading zeros in file name.\n");
    printf ("startIndex             : Index of first image / key file in the sequence of files.\n");
    printf ("endIndex               : Index of last image / key file in the sequence of files.\n");

    printf ("corrsFileDir           : Directory containing pairwise correspondence files.\n");
    printf ("fittingFunction        : Fitting function to be used for outlier rejection (0 - Fundamental matrix; 1 - Homography).\n");

    printf ("----------------------------------------------------------------------\n");
    return false;
  }

  return true;
}

void init1 (int argc, char **argv)
{
  // Parse the command line parameters
  execMode = atoi (argv[1]);
  if (execMode == DETECT_KEYPOINTS)
  {
    keyPtType = (KeyPointType) atoi (argv[2]);
    strImagePrefix = argv[3];
    strImageExtn = argv[4];
    nDigits = atoi (argv[5]);
    nStartIndex = atoi (argv[6]);
    nEndIndex = atoi (argv[7]);
  }
  else if ((execMode == GEN_PAIRWISE_CORRS) || (execMode == GEN_NVIEW_CORRS))
  {
    keyPtType = (KeyPointType) atoi (argv[2]);
    strKeyPrefix = argv[3];
    strKeyExtn = argv[4];
    nDigits = atoi (argv[5]);
    nStartIndex = atoi (argv[6]);
    nEndIndex = atoi (argv[7]);

    if (execMode == GEN_NVIEW_CORRS)
    {
      strPairwiseCorrsDir = argv[8];
      useHomographyRansacer = atoi (argv[9]);
    }
  }
}
void split2(vector<string> &toks, const string &s, const string &delims)
{
	toks.clear();

	string::const_iterator segment_begin = s.begin();
	string::const_iterator current = s.begin();
	string::const_iterator string_end = s.end();

	while (true)
	{
		if (current == string_end || delims.find(*current) != string::npos || *current == '\r')
		{
			if (segment_begin != current)
				toks.push_back(string(segment_begin, current));

			if (current == string_end || *current == '\r')
				break;

			segment_begin = current + 1;
		}

		current++;
	}

}
 int colors[3];


string generate_path(string filename)
{
	vector <string> toks;
		split2(toks,filename,"\\");
		int lasttokenpos = toks.size()-1;
		int lasttokensize = toks[lasttokenpos].size();
		int secondlasttokenpos = toks.size()-2;
		int secondlasttokensize = toks[secondlasttokenpos].size();
		int pathendpos = (filename.size()-lasttokensize)-1;
		string path = filename.substr (0,pathendpos+1);
		return path;
}
string generate_abs_path(string filename)
{
	vector <string> toks;
		split2(toks,filename,"\\");
		int lasttokenpos = toks.size()-1;
		int lasttokensize = toks[lasttokenpos].size();
		int secondlasttokenpos = toks.size()-2;
		int secondlasttokensize = toks[secondlasttokenpos].size();
		int pathendpos = (filename.size()-lasttokensize-secondlasttokensize)-2;
		string path = filename.substr (0,pathendpos+1);
		return path;
}

// Main function of this application.
int main (int argc, char **argv)											/*swati this is the code for generating the
																				file having world points and their
																				* projections in the various views.*/
{
  printf ("main1 - Begin\n");

  int num = atoi(argv[1]);
  char *mapfilename = argv[2];
  char *infilename = argv[3];
  char* outfilename = argv[4];
  char*outfilename2 = argv[5];
 
   colors[0] = stoi(argv[5]);
  colors[1] = stoi(argv[6]);
  colors[2] = stoi(argv[7]);
 string focalfile = string(argv[8]); 
  vector<int>mapping;

  ifstream myfile;
	myfile.open(mapfilename);
	string line;
	while(getline(myfile,line)){
		vector <string> toks;
		split2(toks,line," ");
		mapping.push_back(stoi(toks[0]));
	}
	myfile.close();
	image_to_track.resize(num);

	printf("Corres Generate\n");
  corresGenerate(num, mapping, infilename);
  printf("N-view Generate\n");
  computeNViewCorrs (num);
  printf("Save\n");

  //string path = generate_abs_path(argv[2]);
  string path = generate_path(argv[2]);
  printf("%s\n",path.c_str());
  /*getchar();
  getchar();*/
  vector <string>listofimages;
  string s2;
  myfile.open(focalfile);
	vector < double> camerafocal;
	int numcameras=0;
	while(getline(myfile,s2)) {
		vector <string> toks1;
		split2(toks1, s2, " " );
		listofimages.push_back(toks1[0]);
		
		numcameras++;
	}
	myfile.close();
  
  saveNViewCorrsToDisk (num,outfilename,outfilename2,path,listofimages);
  printf("Save Done\n");
  if (usage1 (argc, argv))
  {
    // Perform any default initializations
    init1 (argc, argv);

    END_INDEX = nEndIndex;
    if (execMode == DETECT_KEYPOINTS)
    {
      isSaveKeySeqToDisk = true;
      //detectKeyPointsInImageSeq ();
    }
    else if (execMode == GEN_PAIRWISE_CORRS)
    {
      isLoadKeySeqFromDisk = true;
      isSavePairWiseCorrsToDisk = true;
     // computePairWiseCorrs ();
    }
    else if (execMode == GEN_NVIEW_CORRS)
    {
      isLoadKeySeqFromDisk = true;
      isLoadPairWiseCorrsToDisk = true;
     // computeNViewCorrs ();
      //saveNViewCorrsToDisk ();
    }
  }

  printf ("main1 - End\n");
  return 0;
}


/** USAGE
Case 1: ./a.out 0 0 ./Images jpg 8 0 1
Case 2: ./a.out 1 0 ./Keys key 8 0 1
Case 3: ./a.out 2 0 ./Keys key 8 0 1 ./correspondingFileDir 0
*/
