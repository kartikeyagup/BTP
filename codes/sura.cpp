#include <jni.h>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <android/log.h>
#include <utility>

#define LOG_TAG    "randomtag"
#define LOG(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

using namespace std;
using namespace cv;
 
int toGray(Mat &img, Mat& gray);

CvRect rect;
Rect region_of_interest;
int test;
Mat src_gray,image,src_gray_prev,src1,src_gray1,copy,copy1,frames,copy2,mask;
int maxCorners = 5000;
RNG rng(12345);
vector<Point2f> corners,corners_prev,corners_temp,cornersinverse;
double qualityLevel = 0.03;
double minDistance = 2;
int blockSize = 7;
bool useHarrisDetector = false;
double k = 0.04;
vector<uchar> status;
int fno=0;
vector<float> err;
string Bigstring="";
int stateoflogging=0;   //0 = initial, 1 = logging, 2= stopped and to write, 3=done
Size winSize(15,15);
int activepoints;
TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.03);
vector< vector<pair<Point2f,Point2f> > > Corrpointpairs;
int HalfDimX=640;
int HalfDimY=360;
float FocalLength = 1150.0;
bool vectorpush=false;
vector<float*> TranslationData;
vector<float*> RotationData;
vector<Mat> ImageCopies;

struct CorrData
{
    Point2f PointData;
    int CameraId;
};

struct TriangPoint
{
   Point3f Pointlocation;
   int colorR;
   int colorG;
   int colorB;
};

vector<vector<CorrData> > CorrespondingPointsData;
//vector<Point3f> TriangulatedPoints;
vector<TriangPoint> TriangulatedPoints;

void TriangulateAll();

int imageclick=0; // Stores the present image which has been clicked

bool sofar=true;

template <typename T>
std::string to_string(T value)
{
    std::ostringstream os ;
    os << value ;
    return os.str() ;
}


void RTMultiply(float* R, float* T)
{
    float t0=T[0];
    float t1=T[1];
    float t2=T[2];
    T[0]=R[0]*t0+R[1]*t1+R[2]*t2;
    T[1]=R[3]*t0+R[4]*t1+R[5]*t2;
    T[2]=R[6]*t0+R[7]*t1+R[8]*t2;
}

void RTransform(float* R)
{
    float t0=R[0];
    float t1=R[1];
    float t2=R[2];
    float t3=R[3];
    float t4=R[4];
    float t5=R[5];

    R[0]=t3;
    R[1]=t4;
    R[2]=t5;
    R[3]=t0;
    R[4]=t1;
    R[5]=t2;
}

void GetGoodPoints(vector<Point2f> &prevtracking, vector<Point2f> &inversetracking, vector<uchar> &status)
{
    for (int i=0; i<prevtracking.size(); i++)
    {
        status[i]=0;
        Point2f temmpPoint = inversetracking[i]-prevtracking[i];
        float magnitude = (temmpPoint.x)*(temmpPoint.x) + (temmpPoint.y)*(temmpPoint.y);
        if (magnitude<=10.0)
        {
            status[i]=1;
        }
    }
}

void Transpose(float* R)
{
    float swap;

    swap=R[1];
    R[1]=R[3];
    R[3]=swap;

    swap=R[2];
    R[2]=R[6];
    R[6]=swap;

    swap=R[5];
    R[5]=R[7];
    R[7]=swap;

}

bool CheckEqualPoints(Point2f p1, Point2f p2)
{
    return ((abs(p1.x - p2.x)<0.01) && (abs(p1.y-p2.y)<0.01));
}


void ConvertPoint(Point2f &p1)
{
    // Converts to center subtracted form
    p1.x = 2*p1.x - HalfDimX;
    p1.y = HalfDimY - 2*p1.y;

    // Multiplying by K inv

    p1.x /= FocalLength;
    p1.y /= FocalLength;
}

float stof(string inp)
{
    float myFloatNumber;
    sscanf(inp.c_str(), "%f", &myFloatNumber);
    return myFloatNumber;
}

vector<float> splitstring(string message)
{
    vector<float> ans;
    int i=0, pos=0, length=0, temp;
    temp = message.find(",",pos);
    while (temp != -1)
    {
        length = temp -pos;
        ans.push_back(stof(message.substr ( pos, length )));
        pos = temp + 1;
        temp = message.find ( ",", pos );
    }
    ans.push_back(stof(message.substr(pos)));
    return ans;
}

string Generate3DToWrite()
{
    string ans="ply\nformat ascii 1.0\nelement vertex ";
    ans += to_string(TriangulatedPoints.size());
    ans+="\nproperty float x\nproperty float y\nproperty float z\nproperty uchar diffuse_red\nproperty uchar diffuse_green\nproperty uchar diffuse_blue\nend_header\n";
    for (int i=0; i<TriangulatedPoints.size() ;i++)
    {
        ans += to_string(TriangulatedPoints[i].Pointlocation.x) + " " + to_string(TriangulatedPoints[i].Pointlocation.y)+" "+to_string(TriangulatedPoints[i].Pointlocation.z)
        + " " + to_string(TriangulatedPoints[i].colorR)+" "+to_string(TriangulatedPoints[i].colorG)+" "+to_string(TriangulatedPoints[i].colorB) + "\n";
    }
    return ans;
}

void Dump3DCloudData(string dstring,string path)
{
    std::ofstream out(path);
    out<<dstring;
    out.close();
}


void LoadDataFromRTFiles(string path)
{
    vector<string> lines;
    string line;
    ifstream myfile (path.c_str());
    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            lines.push_back(line);
        }
        myfile.close();
    }
    for (int i=1;i<lines.size(); i++)
    {
        vector<float> res = splitstring(lines[i]);
        float *newrot = new float[9];
        float *newtrans = new float[3];
        newrot[0]= res[4];
        newrot[1]= res[5];
        newrot[2]= res[6];
        newrot[3]= res[7];
        newrot[4]= res[8];
        newrot[5]= res[9];
        newrot[6]= res[10];
        newrot[7]= res[11];
        newrot[8]= res[12];

        newtrans[0] = res[1];
        newtrans[1] = res[2];
        newtrans[2] = res[3];

//        RTransform(newrot);
//        Transpose(newrot);
        RTMultiply(newrot,newtrans);

        TranslationData.push_back(newtrans);
        RotationData.push_back(newrot);
    }

    __android_log_print(ANDROID_LOG_INFO,"yellortdata","Completed RT DATA retreival");
    TriangulateAll();
    string D3output = Generate3DToWrite();
    Dump3DCloudData(D3output,path+".ply");
}

void Triangulate(vector<CorrData> &inputpoints)
{
    int num_points = inputpoints.size();
    __android_log_print(ANDROID_LOG_INFO,"yellotriangdata","Starting triangulation for point is: %d", num_points);
    int num_eqs = 2*num_points;
    int num_vars = 3;

    int colr,colg,colb;

    int imagef = inputpoints[0].CameraId;
    Point2f pointcoord = inputpoints[0].PointData;

//    Convert points opencv coordinates to center subtracted
    float posx = 2 *(pointcoord.x);
    float posy = 2 *(pointcoord.y);

    Vec3b colour = ImageCopies[imagef].at<Vec3b>(Point(posx,posy));
    colr = colour.val[2];
    colg = colour.val[1];
    colb = colour.val[0];

    Mat A(num_eqs,3,CV_32F);
    Mat B(num_eqs,1,CV_32F);
    Mat X(3,1,CV_32F);
    for (int i=0; i<num_points;i++)
    {
        float* rot = RotationData[inputpoints[i].CameraId];
        float* tr = TranslationData[inputpoints[i].CameraId];
        Point2f prespoint = inputpoints[i].PointData;
        ConvertPoint(prespoint);

        int row = 2 * i;

//        A.data[0]

        A.at<float>(row,0) = rot[0] - prespoint.x*rot[6];
        A.at<float>(row,1) = rot[1] - prespoint.x*rot[7];
        A.at<float>(row,2) = rot[2] - prespoint.x*rot[8];

        A.at<float>(row + 1 ,0) = rot[3] - prespoint.y*rot[6];
        A.at<float>(row + 1 ,1) = rot[4] - prespoint.y*rot[7];
        A.at<float>(row + 1 ,2) = rot[5] - prespoint.y*rot[8];

        B.at<float>(row + 0,0) = tr[2]*prespoint.x - tr[0];
        B.at<float>(row + 1,0) = tr[2]*prespoint.y - tr[1];
//        __android_log_print(ANDROID_LOG_INFO,"yellotriangdata","%f x + %f y + %f z = %f", A.at<float>(row,0),A.at<float>(row,1),A.at<float>(row,2),B.at<float>(row,0) );
//        __android_log_print(ANDROID_LOG_INFO,"yellotriangdata","%f x + %f y + %f z = %f", A.at<float>(row+1,0),A.at<float>(row+1,1),A.at<float>(row+1,2),B.at<float>(row+1,0) );

//        __android_log_print(ANDROID_LOG_INFO,"yellotriangdata","%f x + %f y + %f z = %f", A.at<);

//        __android_log_print(ANDROID_LOG_INFO,"yellotriangdata","Got all data for equations for triangulation for point is: %d", num_points);
    }
    __android_log_print(ANDROID_LOG_INFO,"yellotriangdata","Got all data for equations for triangulation for point is: %d", num_points);
    cv::solve(A,B,X,DECOMP_SVD);
    Point3f newp;
    newp.x = X.at<float>(0,0);
    newp.y = X.at<float>(1,0);
    newp.z = X.at<float>(2,0);

    TriangPoint newt;
    newt.Pointlocation = newp;
    newt.colorR = colr;
    newt.colorG = colg;
    newt.colorB = colb;

    __android_log_print(ANDROID_LOG_INFO,"yellotriangdata","Completed triangulation and point is: %f, %f, %f ",newp.x, newp.y,newp.z);

//    TriangulatedPoints.push_back(newp);
    TriangulatedPoints.push_back(newt);
    //Opencv Solve function
}

void TriangulateAll()
{
    for (int i=0; i< CorrespondingPointsData.size(); i++)
    {
        Triangulate(CorrespondingPointsData[i]);
    }
}



void UpdateCorrespondance(vector<vector<CorrData>> &CorrPoints, vector<pair<Point2f, Point2f>> &FramePoints, int fno)
{
    int previndex=0;
    for (int i=0;i<FramePoints.size();i++)
    {
        pair<Point2f, Point2f> present=FramePoints[i];
        while(previndex<CorrPoints.size())
        {
            if (CorrPoints[previndex].back().CameraId == fno)
            {
//                if (present.first == CorrPoints[previndex].back().PointData) {
                 __android_log_print(ANDROID_LOG_INFO,"corrdebug","in the same camera id");
                if ( CheckEqualPoints(present.first,CorrPoints[previndex].back().PointData))
                {

                 __android_log_print(ANDROID_LOG_INFO,"corrdebug","found global corr");
                    CorrData n1;
                    n1.PointData = present.second;
                    n1.CameraId = fno + 1;
                    CorrPoints[previndex].push_back(n1);
                    break;
                }
            }
            previndex+=1;
        }
        if (previndex>=CorrPoints.size())
        {
            for (int k=i;k<FramePoints.size(); k++) {
                CorrData n1;
                n1.PointData = FramePoints[k].first;
                n1.CameraId = fno;
                CorrData n2;
                n2.PointData = FramePoints[k].second;
                n2.CameraId = fno + 1;
                vector <CorrData> newvect;
                newvect.push_back(n1);
                newvect.push_back(n2);
                CorrPoints.push_back(newvect);
            }
            break;
        }
    }
}

string GetCorresString(vector<CorrData> &CorrLine)
{
    string ans="";
    for (int i=0; i<CorrLine.size(); i++)
    {
        ans+=","+to_string(CorrLine[i].CameraId)+","+to_string(CorrLine[i].PointData.x)+","+to_string(CorrLine[i].PointData.y);
    }
    return ans.substr(1);
}

string GetBigStringForCorres(vector<vector<CorrData> > &CorrDataTotal)
{
    string bigans="";
    for (int k=0;k<CorrDataTotal.size(); k++)
    {
        bigans+=GetCorresString(CorrDataTotal[k])+"\n";
    }
    return bigans;
}



extern "C" {

vector< pair<Point2f,Point2f> > MakeVectorPair(vector<Point2f> &vect1, vector<Point2f> &vect2)
{
    vector< pair<Point2f, Point2f>> ans;
    for (int i=0; i<vect1.size(); i++)
    {
        ans.push_back(pair<Point2f,Point2f> (vect1[i],vect2[i]));
    }
    return ans;
}

void getCorrPoints(int id, vector<Point2f> &previouspoints, vector<Point2f> &presentpoints, vector<uchar> &statusarr)
{
    int l1= Corrpointpairs[id].size();
    vector< pair<Point2f, Point2f> > ans;
    int prevval=0;
    __android_log_print(ANDROID_LOG_INFO,"debugcorr","vectsize: %d, previouspointssize:%d presentpointssize:%d statussize:%d",Corrpointpairs[id].size(),previouspoints.size(),presentpoints.size(),statusarr.size());
    for (int i=0;( i<statusarr.size() && prevval<l1); i++)
    {
        if (statusarr[i]!=0)
        {
//            while (Corrpointpairs[id][prevval].second != previouspoints[i])
//            __android_log_print(ANDROID_LOG_INFO,"debugcorr","i :%d, prevva: %d" ,i, prevval);
            while (!CheckEqualPoints(Corrpointpairs[id][prevval].second,previouspoints[i]))
            {
//                __android_log_print(ANDROID_LOG_INFO,"debugcorr","trackpoint :%f,%f corrpoint: %f,%f" ,previouspoints[i].x,previouspoints[Corrpointpairs[id]);
                prevval+=1;
                if (prevval>=l1)
                {
                    break;
                }
            }
//            __android_log_print(ANDROID_LOG_INFO,"debugcorr","i :%d, prevva: %d" ,i, prevval);
            ans.push_back(pair<Point2f, Point2f> (Corrpointpairs[id][prevval].first,presentpoints[i]));
            prevval +=1;
        }
    }
//    for(int i=0;i<ans.size();i++)
    __android_log_print(ANDROID_LOG_INFO,"debugcorr","ans:%d",ans.size());
    Corrpointpairs[id]=ans;
}

string GetRelevantString(vector<Point2f> &prev,vector<Point2f> &present, vector<uchar> &statusarr)
{
    string anstop="";
    string ansbottom="";
    for (int i=0; i<statusarr.size(); i++)
    {
        if (statusarr[i]!=0)
        {
            anstop += ","+to_string(prev[i].x) +","+to_string(prev[i].y);
            ansbottom += ","+to_string(present[i].x) +","+to_string(present[i].y);
        }
    }
    anstop= anstop.substr(1) + "\n" + ansbottom.substr(1) +"\n";
    return anstop;
}

string GetRelStringCorrPointPairs(vector<pair<Point2f,Point2f> >  &corrpairs)
{
    string anstop="";
    string ansbottom="";
    for (int i=0;i<corrpairs.size(); i++)
    {
        anstop += "," + to_string(corrpairs[i].first.x)+ "," + to_string(corrpairs[i].first.y);
        ansbottom += "," + to_string(corrpairs[i].second.x)+ "," + to_string(corrpairs[i].second.y);
    }
    anstop= anstop.substr(1)+ "\n"+ ansbottom.substr(1)+"\n";
    return anstop;
}

string GetBigStringCorPointPairs(vector<vector<pair<Point2f,Point2f> > > &bigpairs)
{
    string bigans="";
    for (int i=0;i<bigpairs.size(); i++)
    {
        bigans += GetRelStringCorrPointPairs(bigpairs[i]);
    }
    return bigans;
}

void WriteDataToFile(string path)
{
    std::ofstream out(path);
    out<<Bigstring;
    out.close();
}

void WriteTrackDataToFile(string path)
{
    std::ofstream out(path);
    string strtowrite = GetBigStringCorPointPairs(Corrpointpairs);
    out<<strtowrite;
    out.close();
}

void WriteCorresDataToFile(string path)
{
    std::ofstream out(path);
    string strtowrite = GetBigStringForCorres(CorrespondingPointsData);
    out<<strtowrite;
    out.close();
}

JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_convertNativeGray(JNIEnv*, jobject, jlong addrRgba, jlong addrGray);

JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_startNativeLogging(JNIEnv*, jobject);

JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_startImage(JNIEnv*, jobject);

JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_stopImage(JNIEnv*, jobject);

JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_stopNativeLogging(JNIEnv*, jobject, jstring);

JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_startTriangulation(JNIEnv*,jobject,jstring);

JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_startNativeLogging(JNIEnv*, jobject)
{
//    Corrpointpairs.push_back(MakeVectorPair(corners_prev,corners_prev));
    vectorpush=true;
    __android_log_print(ANDROID_LOG_INFO,"yellocorr","Started the image and copied the base pair, %d of size",imageclick);
    stateoflogging=1;
    return (jint) 0;
}


JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_startImage(JNIEnv*, jobject)
{
//    Corrpointpairs.push_back(MakeVectorPair(corners_prev,corners_prev));
    vectorpush=true;
    imageclick++;
    __android_log_print(ANDROID_LOG_INFO,"yellocorr","Started the image and copied the base pair, %d of size",imageclick);
//    if (imageclick==2)
//    {
//        for (int l=0; l<Corrpointpairs[1].size();l++)
//        {
//            vector<CorrData> v1;
//            CorrData n1;
//            CorrData n2;
//            n1.CameraId=1;
//            n2.CameraId=2;
//            n1.PointData=Corrpointpairs[1][l].first;
//            n2.PointData=Corrpointpairs[1][l].second;
//            v1.push_back(n1);
//            v1.push_back(n2);
//            CorrespondingPointsData.push_back(v1);
//        }
//    }
//    else if(imageclick>2)
//    {
//       UpdateCorrespondance(CorrespondingPointsData,Corrpointpairs[imageclick-1],imageclick-1);
//    }
    __android_log_print(ANDROID_LOG_INFO,"yello1","Started the image and copied the base pair, %d",imageclick);
    return (jint) 0;
}

JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_stopImage(JNIEnv*, jobject)
{
    imageclick=0;
    return (jint) 0;
}

JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_stopNativeLogging(JNIEnv* env, jobject, jstring path)
{
    const char* stringpassed= env->GetStringUTFChars(path,JNI_FALSE);
    string bigpath=stringpassed;
    stateoflogging=2;
    WriteDataToFile(bigpath+"_track.csv");
    WriteTrackDataToFile(bigpath+"_corrpairs.csv");
    WriteCorresDataToFile(bigpath+"_globalcorres.csv");
    stateoflogging=3;
    return (jint) 0;
}


JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_startTriangulation(JNIEnv* env,jobject,jstring path)
{
    const char* stringpassed = env->GetStringUTFChars(path,JNI_FALSE);
    string pathconv = stringpassed;
    LoadDataFromRTFiles(pathconv);
    // START TRIANGULATION
}

JNIEXPORT jint JNICALL Java_org_opencv_sura_MainActivity_convertNativeGray(JNIEnv*, jobject, jlong addrRgba, jlong addrGray)
{
    Mat& mRgb = *(Mat*)addrRgba;
    Mat& mGray = *(Mat*)addrGray;
    
    int conv;
    jint retVal;    
    
    conv = toGray(mRgb, mGray);
    retVal = (jint)conv;
 
    return retVal;
}
 
}
// template <typename T>
// std::string to_string(T value)
// {
//     std::ostringstream os;
//     os <<value;
//     return os.str();
// }
int toGray(Mat &img, Mat& gray)
{
    fno+=1;
    cvtColor(img, gray, CV_RGBA2GRAY); // Assuming RGBA input
    resize(gray,src_gray,Size(),0.5,0.5);
    if (sofar)
    {
//        cv::Mat mask = cv::Mat::zeros(src_gray.size(), CV_8UC1);
//        cv::Mat roi(mask, cv::Rect(x_point,y_point,width_point,height_point));
//        roi = cv::Scalar(255, 255, 255);
//        Mat copy;
//        copy = src_gray.clone();
        goodFeaturesToTrack( src_gray,
                         corners,
                         maxCorners,
                         qualityLevel,
                         minDistance,
                         mask,
                         blockSize,
                         useHarrisDetector,
                         k );
        src_gray.copyTo(src_gray_prev);
        corners_prev = corners;
        if(corners_prev.size()>0)
        {    sofar=false;
        }
   }

    else
    {
        string s=to_string(corners_prev.size());
//       LOG("hello %s",s);
        __android_log_print(ANDROID_LOG_INFO,"yello","test str = %d",corners_prev.size());
        if (corners_prev.size()>0)
        {
            calcOpticalFlowPyrLK(src_gray_prev, src_gray, corners_prev, corners, status, err,winSize, 2, termcrit, 0, 0.001);
            calcOpticalFlowPyrLK(src_gray,src_gray_prev,corners,cornersinverse,status,err,winSize,2,termcrit,0,0.001);

            GetGoodPoints(corners_prev,cornersinverse,status);

            src_gray.copyTo(src_gray_prev);

            if (stateoflogging==1)
            {
//                __android_log_print(ANDROID_LOG_INFO,"yello123","Started the image and copied the base pair, %d",imageclick);
                if(!vectorpush)
                {
//                  __android_log_print(ANDROID_LOG_INFO,"yello123","Started the image and copied the base pair, %d",imageclick);

                    getCorrPoints(imageclick,corners_prev,corners,status);
                }
                else if (imageclick>0)
                {
                    getCorrPoints(imageclick-1,corners_prev,corners,status);
                }
                Bigstring += GetRelevantString(corners_prev,corners,status);
            }

            corners_prev.clear();
            activepoints=0;
            for(int i=0;i<status.size();i++)
            {
                if(status[i]!=0)
                {
//                  activepoints++;
                    corners_prev.push_back(corners[i]);
                }
            }
            if(vectorpush)
            {
                vectorpush=false;
                Corrpointpairs.push_back(MakeVectorPair(corners_prev,corners_prev));

                Mat copimg;
                copimg = img.clone();
                ImageCopies.push_back(copimg);

//                if (imageclick==2)
//                    {
//                        for (int l=0; l<Corrpointpairs[2].size();l++)
//                        {
//                            vector<CorrData> v1;
//                            CorrData n1;
//                            CorrData n2;
//                            n1.CameraId=1;
//                            n2.CameraId=2;
//                            n1.PointData=Corrpointpairs[2][l].first;
//                            n2.PointData=Corrpointpairs[2][l].second;
//                            v1.push_back(n1);
//                            v1.push_back(n2);
//                            CorrespondingPointsData.push_back(v1);
//                        }
//                    }
                    if(imageclick>=2)
                    {
                        // Some error here, put in imageclick and test. Not sure.
                       UpdateCorrespondance(CorrespondingPointsData,Corrpointpairs[imageclick-1],imageclick-2);
                    }
            }
        }
//        __android_log_print(ANDROID_LOG_INFO,"yello","active points = %d",corners_prev.size());
//       if(activepoints<5)
//       {
//                goodFeaturesToTrack( src_gray,
//                                         corners,
//                                         maxCorners,
//                                         qualityLevel,
//                                         minDistance,
//                                         mask,
//                                         blockSize,
//                                         useHarrisDetector,
//                                         k );
//                src_gray.copyTo(src_gray_prev);
//                corners_prev = corners;
//       }
    }

    if ((fno%10==0) )
    {
        cv::Mat mask= cv::Mat::ones(src_gray.size(), CV_8UC1);
        for (int i=0; i<corners_prev.size(); i++)
        {
            cv::circle(mask, Point2f(corners_prev[i].x,corners_prev[i].y), 3, cv::Scalar( 0 ), -1 );
        }
        vector<Point2f> newcorners;
        goodFeaturesToTrack(src_gray,
                                newcorners,
                                maxCorners,
                                qualityLevel,
                                minDistance,
                                mask,
                                blockSize,
                                useHarrisDetector,
                                k);
        corners_prev.insert(corners_prev.end(),newcorners.begin(),newcorners.end());
        corners.insert(corners.end(),newcorners.begin(),newcorners.end());
    }


    for( size_t i = 0; i < corners_prev.size(); i++ )
    {
        Point2f temp;
        temp.x=2*corners_prev[i].x;
        temp.y=2*corners_prev[i].y;
        cv::circle( gray, temp, 10, cv::Scalar( 255. ), -1 );
    }

    cv::Point textOrg(10,130);
    cv::putText(gray,to_string(corners_prev.size()), textOrg , FONT_HERSHEY_SCRIPT_SIMPLEX, 2, cv::Scalar(255));
//    oldimgclick=imageclick;
    return 1;
}
