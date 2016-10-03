#include <stdio.h>

#include "utils.h"
#include <vector>
#include<map>
#include <list>

#include "CorrespondenceGenerator.h"

#define MIN_MATCHES_REQUIRED (8)
int detectASiftKeyPoints (char *filename, VecOfKeyPointWithDesc& keyPtDescVec);
const int DESC_PER_LINE = 20;

TableOfKeyPointWithDesc keyPointTable;      /**< Structure maintaining key points for each image. */
//VecOfKeyPointMatch *pPairwiseKeyPointMatches;         /**< Structure maintaining pairwise key point matches. */
//VecOfFeatureList corrsTable;
using namespace std;
vector<vector<pair<pair<int,int>,pair<double,double >> > > corrsTable;

int nPairwiseMatches = 0;

int END_INDEX;

// Detects key points for an image sequence


// Computes pair wise correspondences for an image sequence


// Computes N-View correspondences for an image sequence
using namespace std;
#include <hash_map>
#include <hash_set>
typedef std::pair<unsigned long, unsigned long> MatchIndex1;


#ifdef WIN32
namespace stdext {
    template<>
    class hash_compare<MatchIndex1> {
	public:
		static const size_t bucket_size = 4;
        static const size_t min_buckets = 8;
        size_t
        operator()(const MatchIndex1 &__x) const
        { return __x.first * 1529 + __x.second; }

        bool operator()(const MatchIndex1 &__x1, const MatchIndex1 &__x2) const {
			return (__x1.first < __x2.first) || (__x1.first == __x2.first && __x1.second < __x2.second);
        }
    };
}
#else
namespace __gnu_cxx {
    template<>
    struct hash<MatchIndex1> {
        size_t
        operator()(MatchIndex1 __x) const
        { return __x.first * 1529 + __x.second; }
    };
}
#endif
 stdext::hash_map<MatchIndex1, vector<pair<int,int>> > pPairwiseKeyPointMatches;
 stdext::hash_map<MatchIndex1, vector<pair<int,int>> >::iterator it;
 vector<vector<pair<int,pair<int,int>>>>image_to_track;
void computeNViewCorrs (int nViews)
{
  printf ("computeNViewCorrs - Begin\n");

  // Load key point file sequence from disk
 // if (isLoadKeySeqFromDisk)
   // loadKeyPointSeqFromDisk ();

  // Load pairwise key point matches from disk
  //if (isLoadPairWiseCorrsToDisk)
    //loadPairwiseCorrsFromDisk ();

  // For each pair of consecutive images
  int idx1, idx2;
  //int nViews = END_INDEX - nStartIndex + 1;
 printf("%d\n",pPairwiseKeyPointMatches.size());
 int count = 0;
 //getchar();
 vector<map<int,int>> maprows;

 FeatureList tmpFeatureList = new Feature [nViews];
  for (it = pPairwiseKeyPointMatches.begin(); it != pPairwiseKeyPointMatches.end() ; it++)
    {
      // Retrieving list of good matches
      int nGoodMatches = 0, nMatches = 0;
      bool *goodMatchIndexArr = NULL;
	  printf("%d\n",count);
	  
      // Ensure that number of key point matches is more than the lower limit
	  nMatches = it->second.size ();

	  if(count == 6036)
	  {
		  printf("%d\n",nMatches);
	  }
      if (nMatches > MIN_MATCHES_REQUIRED)
      {
       // goodMatchIndexArr = new bool[nMatches];
        //getListOfGoodMatches (curPairIndex, i, j, nGoodMatches, &goodMatchIndexArr);
      
        if (1)//nGoodMatches >= MIN_MATCHES_REQUIRED)
        {
          for (int k = 0; k < nMatches; k++)
          {
            
			  int m1 = it->first.first;
			  int m2 = it->first.second;
            // Retreiving matching key indices
			  idx1 = it->second[k].first;
			  idx2 = it->second[k].second;

			  if(count == 6036)
			  {
				  printf("%d %d %d %d\n",m1,m2,idx1,idx2);
			  }

            KeyPointWithDesc keyPt1 = keyPointTable[m1][idx1];
            KeyPointWithDesc keyPt2 = keyPointTable[m2][idx2];

            // Both the points are not already present in the correspondence table
            if ((keyPt1.m_extra == -1) && (keyPt2.m_extra == -1))
            {
              // Creating a new correspondence list
              // printf ("main - Creating a new correspondence list\n");

				if(count == 6036)
				  {
					  printf("%d\n",nViews);
				  }
              vector<pair<pair<int,int>,pair<double,double>>>tmp;
			  pair<pair<int,int>,pair<double,double>>tmp2;

             /* for (int viewIndex = 0; viewIndex < nViews; viewIndex++)
              {
                tmpFeatureList[viewIndex].x = -10000.0;
                tmpFeatureList[viewIndex].y = -10000.0;
              }*/

              // Adding first image's keypoint to the correspondence list
             // tmpFeatureList[m1].x = keyPt1.m_x;
            //  tmpFeatureList[m1].y = keyPt1.m_y;
			  tmp2.first.first = m1;
			  tmp2.first.second = idx1;
			  tmp2.second.first = keyPt1.m_x;
			  tmp2.second.second = keyPt1.m_y;
			  tmp.push_back(tmp2);
              // Adding second image's keypoint to the correspondence list
            //  tmpFeatureList[m2].x = keyPt2.m_x;
            //  tmpFeatureList[m2].y = keyPt2.m_y;

			  tmp2.first.first = m2;
			  tmp2.first.second = idx2;
			  tmp2.second.first = keyPt2.m_x;
			  tmp2.second.second = keyPt2.m_y;
			  tmp.push_back(tmp2);

			  map<int,int> tmpmap;

			  tmpmap.insert(make_pair(m1,idx1));
			  tmpmap.insert(make_pair(m2,idx2));
			  maprows.push_back(tmpmap);
              corrsTable.push_back (tmp);
			  tmp.clear();
              keyPointTable[m1][idx1].m_extra = corrsTable.size () - 1;
              keyPointTable[m2][idx2].m_extra = corrsTable.size () - 1;
			  pair <int,pair<int,int>> tmpC;
			  tmpC.first = corrsTable.size () - 1;
			  tmpC.second.first=keyPt1.m_x;
			  tmpC.second.second=keyPt1.m_y;
			  image_to_track[m1].push_back(tmpC);

			 // delete[] tmpFeatureList;
            }
            else
            {
              // keyPt1 is present in the table
              if (keyPt2.m_extra == -1)
              { // Add keyPt2 to correspondence table and update it's extra flag to point to correct row in corresondence table
               // corrsTable[keyPt1.m_extra][m2].x = keyPt2.m_x;
               // corrsTable[keyPt1.m_extra][m2].y = keyPt2.m_y;
              
				 vector<pair<pair<int,int>,pair<double,double>>>tmp;
			  pair<pair<int,int>,pair<double,double>>tmp2;
				map<int,int> &tmpmap = maprows[keyPt1.m_extra];
				map <int,int>::iterator it1;
				it1 = tmpmap.find(m2);
				if(it1 == tmpmap.end())
				{
				  tmp2.first.first = m2;
				  tmp2.first.second = idx2;
				  tmp2.second.first = keyPt2.m_x;
				  tmp2.second.second = keyPt2.m_y;
				  corrsTable[keyPt1.m_extra].push_back(tmp2);


					keyPointTable[m2][idx2].m_extra = keyPt1.m_extra;
					tmpmap.insert(make_pair(m2,idx2));
				}
              }
              else  // keyPt2 is present in the table
              { // Add keyPt1 to correspondence table and update it's extra flag to point to correct row in corresondence tablee
                //corrsTable[keyPt2.m_extra][m1].x = keyPt1.m_x;
               // corrsTable[keyPt2.m_extra][m1].y = keyPt1.m_y;
				  if(keyPt1.m_extra == -1)
				  {
					  vector<pair<pair<int,int>,pair<double,double>>>tmp;
						  pair<pair<int,int>,pair<double,double>>tmp2;
							map<int,int> & tmpmap = maprows[keyPt2.m_extra];
							map <int,int>::iterator it1;
							it1 = tmpmap.find(m1);
							if(it1 == tmpmap.end())
							{
							  tmp2.first.first = m1;
							  tmp2.first.second = idx1;
							   tmp2.second.first = keyPt1.m_x;
							   tmp2.second.second = keyPt1.m_y;
								int val = keyPt2.m_extra;
								corrsTable[val].push_back(tmp2);

								keyPointTable[m1][idx1].m_extra = keyPt2.m_extra;
								tmpmap.insert(make_pair(m1,idx1));
							}
				  }
              }
            }
          }
        }
      }

      if (goodMatchIndexArr) { delete[] goodMatchIndexArr; goodMatchIndexArr = NULL; }
     // ++curPairIndex;
  count++;  
  }
  
  delete[] tmpFeatureList;
  printf ("computeNViewCorrs - End\n");
}

// Clears the key point table
void clearKeyPointTable ()
{
  printf ("clearKeyPointTable - Begin\n");
  // Clear any memory already used by key point table
  int nKeyPointLists = keyPointTable.size ();
  for (int i = 0; i < nKeyPointLists; i++)
    keyPointTable[i].clear ();

  keyPointTable.clear ();
  printf ("clearKeyPointTable - End\n");
}

// Clears the in-memory pair wise matches storage


// Clears the correspondence table
void clearCorrsTable ()
{
  printf ("clearCorrsTable - Begin\n");
  int nWorldPts = corrsTable.size ();
 // for (int i = 0; i < nWorldPts; i++)
   // SAFE_DELETE_PTR (corrsTable[i]);

  corrsTable.clear ();
  printf ("clearCorrsTable - End\n");
}

// Detects SIFT key points on the input image





// Retrieves the list of good matches


// Loads a correspondence table from disk


void saveNViewCorrsToDisk (int nViews,char*outfilename, char*trackfile, string path, vector <string> listfile )
{
  // Opening the world point projections file
 // int nViews = END_INDEX - nStartIndex + 1;
  FILE *fpOut = fopen (outfilename, "w");
   FILE *fpOut2 = fopen (trackfile, "w");
  if (fpOut)
  {
	vector <vector <int>> colorcorrsTable; 
    double x, y;
    double zero = 0.0;
    int nCorrespondences = corrsTable.size ();
	colorcorrsTable.resize(nCorrespondences);
	for (int i=0;i<colorcorrsTable.size();i++)
	{
		colorcorrsTable[i].resize(3);
	}
	int tracks_coloured=0;
	for (int i=0;i<image_to_track.size();i++)
	{
		if (image_to_track[i].size() > 0)
		{
			string imagename1 = path + listfile[i];
			/*printf("%s\n",imagename1.c_str());
			getchar();*/
			cv::Mat image1 = cv::imread(imagename1);
			
			for (int j=0;j< image_to_track[i].size();j++)
			{
			
				int pixel_x = image_to_track[i][j].second.first + image1.cols/2;
				int pixel_y =  image_to_track[i][j].second.second+ image1.rows/2;
				/*printf("%d %d\n", pixel_x,pixel_y);
				fflush(stdout);*/
				/*CvScalar s = cvGet2D(image1, pixel_y, pixel_x);*/
				int tmp_trackid=image_to_track[i][j].first;
				colorcorrsTable[tmp_trackid][0]=image1.at<cv::Vec3b>(pixel_y,pixel_x)[2];
				colorcorrsTable[tmp_trackid][1]=image1.at<cv::Vec3b>(pixel_y,pixel_x)[1];
				colorcorrsTable[tmp_trackid][2]=image1.at<cv::Vec3b>(pixel_y,pixel_x)[0];
				tracks_coloured++;
			}
		
		
		}
	
	
	}
	printf("%d %d\n",tracks_coloured,nCorrespondences);
		//	getchar();
	if(tracks_coloured != nCorrespondences){
		printf("something terribly wrong\n");
	}

    printf ("saveNViewCorrsToDisk - Writing nCorrespondences = %d\n", nCorrespondences);
    for (int i = 0; i < nCorrespondences; i++)
    {
      // Find the number of valid correspondences
      int nValidCorrs = 0;
      /*for (int j = 0; j < nViews; j++)
      {
        if ((corrsTable[i][j].x != -10000.0) && (corrsTable[i][j].y != -10000.0))
          ++nValidCorrs;
      }*/

      // Write the world point estimate = (0, 0, 0)
      fprintf (fpOut, "%.2lf %.2lf %.2lf %d %d %d ", zero, zero, zero,colorcorrsTable[i][0],colorcorrsTable[i][1],colorcorrsTable[i][2]);

      // Write the number of valid correspondences available
	  fprintf (fpOut, "%d ", corrsTable[i].size());
	  
      // Iterate through the correspondence list
      for (int j = 0; j < corrsTable[i].size(); j++)
      {
        //if ((corrsTable[i][j].x != -10000.0) && (corrsTable[i][j].y != -10000.0))
        {
          //  Write the index of the image and its location in that image
          fprintf (fpOut, "%d %d %.2lf %.2lf ", 
			  corrsTable[i][j].first.first,corrsTable[i][j].first.second, (double) corrsTable[i][j].second.first, (double) corrsTable[i][j].second.second);
        }
      }

      fprintf (fpOut, "\n");
    }

    fclose (fpOut);
  }


  for(int i = 0; i < image_to_track.size(); i++)
  {
	  for(int j = 0; j < image_to_track[i].size(); j++)
	  {
		  fprintf(fpOut2,"%d ",image_to_track[i][j]);
	  }
	  fprintf(fpOut2,"\n");
  }
  fclose(fpOut2);
}

// Initialize internal structures from in-memory correspondence table structure.


void split(vector<string> &toks, const string &s, const string &delims)
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

void corresGenerate(int numImage,vector<int>&mapping, char*filename)
{
	 keyPointTable.resize (numImage);
	 int im1,im2,numpoint,idx1,idx2;
	 double val1,val2,val3,val4;
	 ifstream mylist1;
	 string s2;
	 vector<string>toks;

	 
  

	 //mylist1.open("G:\\BROJO\\one-shot_framewrks\\codes_and_data_trans_avg\\collosseo\\cluster3\\matches_forRT.txt");
	// mylist1.open("G:\\BROJO\\one-shot_framewrks\\codes_and_data_trans_avg\\trans_avg_iterative_least_squares\\matches_forRTA_filtered.txt");
	 mylist1.open(filename);
	 getline(mylist1,s2);
	 split(toks,s2," ");
	 int nPairwiseMatches = stoi(toks[0]);
	// pPairwiseKeyPointMatches = new VecOfKeyPointMatch[nPairwiseMatches];
	// pPairwiseKeyPointMatches = new vector<pair<int,int>> * [nPairwiseMatches];
	vector<string>listA;
	for(int i = 0; i <numImage; i++)
		keyPointTable[i].resize(140000);
	
	int curPairIndex=0;
	int num_p = 0;
	while (getline(mylist1,s2)) {
		
		split(toks,s2," ");
		im1 = mapping[stoi(toks[0])];
		im2 = mapping[stoi(toks[1])];
		if(num_p%100000 == 0)
			printf("%d\n",num_p);
		
		//im1 = stoi(toks[0]);
		//im2 = stoi(toks[1]);
		if(num_p== 160363 )
		{
			printf("hihi");
		}
		
		//getline(mylist1,s2);
		//split(toks,s2," ");
		 numpoint = stoi(toks[2]);
		// printf("%d\n",numpoint);
		 for(int n = 0; n < numpoint; n++)
		 {
			 getline(mylist1,s2);
			split(toks,s2," ");
			idx1 = stoi(toks[0]);
			val1 = stod(toks[1]);
			val2 = stod(toks[2]);

			idx2 = stoi(toks[3]);
			val3 = stod(toks[4]);
			val4 = stod(toks[5]);

			if(num_p == 160363)
			{
			//	printf("%d %d %d %d\n",im1,im2,idx1,idx2);
			}
			keyPointTable[im1][idx1].m_extra=-1;
			keyPointTable[im1][idx1].m_x= val1;
			keyPointTable[im1][idx1].m_y= val2;

			keyPointTable[im2][idx2].m_extra=-1;
			keyPointTable[im2][idx2].m_x= val3;
			keyPointTable[im2][idx2].m_y= val4;

			 KeyPointMatch tmpKeyPointMatch (idx1, idx2);
			 MatchIndex1 midx = MatchIndex1((unsigned long) im1, (unsigned long) im2);
			 //pPairwiseKeyPointMatches[midx].first =idx1;
			 //pPairwiseKeyPointMatches[midx].second =idx2;
			 pPairwiseKeyPointMatches[midx].push_back(make_pair(idx1,idx2));
			  curPairIndex++;
		 }
		 num_p++;
	}
	mylist1.close();
	mapping.clear();
	
	
	printf("Done\n");
}




