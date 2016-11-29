#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core.hpp>
#include "opencv2/core/types_c.h"

using namespace std;
using namespace cv;

class CVec2d
{
public:
  double x;
  double y;
  
  CVec2d()
  {
    x = 0.0;
    y = 0.0;
  }
  
  CVec2d(double p, double q)
  {
    x = p;
    y = q;
  }
  
  ~CVec2d() {};
  
  double Norm()
  {
    return(sqrt(x*x + y*y));
  }

  CVec2d Unit()
  {
    double norm = sqrt(x*x + y*y);
    CVec2d tmp(x / norm, y / norm);
      return tmp;
  }
};

class CVec2f
{
public:
  double x;
  double y;
  
  CVec2f()
  {
    x = 0.0;
    y = 0.0;
  }
  
  CVec2f(double p, double q)
  {
    x = p;
    y = q;
  }
  
  ~CVec2f() {};
  
  double Norm()
  {
    return(sqrt(x*x + y*y));
  }

  CVec2f Unit()
  {
    double norm = sqrt(x*x + y*y);
    CVec2f tmp(x / norm, y / norm);
    return tmp;
  }
};

class CVec3d
{
public:
  double x;
  double y;
  double z;
  
  CVec3d()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
  
  CVec3d(double p, double q, double r)
  {
    x = p;
    y = q;
    z = r;
  }
  
  ~CVec3d() {};
  
  double Norm()
  {
    return(sqrt(x*x + y*y + z*z));

  }
  CVec3d Unit()
  {
    double norm = sqrt(x*x + y*y + z*z);
    CVec3d tmp(x/norm,y/norm,z/norm);
    return tmp;
  }

  CVec3d Cross(CVec3d v1)
  {
    CVec3d tmp;
    tmp.x = y*v1.z - z*v1.y; // u2v3 - u3v2
    tmp.y = z*v1.x - x*v1.z; //u3v1 - u1v3
    tmp.z = x*v1.y - y*v1.x; //u1*v2 - u2*v1
    return tmp;
  }

  double operator*(const CVec3d& rhs) // compound assignment (does not need to be a member,
  {                           // but often is, to modify the private members)
                /* addition of rhs to *this takes place here */
    double val;
    val = this->x*rhs.x + this->y*rhs.y + this->z*rhs.z;
    return val;
  }

  CVec3d operator/(const double rhs) // compound assignment (does not need to be a member,
  {                           // but often is, to modify the private members)
                /* addition of rhs to *this takes place here */
    CVec3d val;
    val.x = this->x / rhs;
    val.y = this->y / rhs;
    val.z= this->z / rhs;
    return val;
  }

  CVec3d operator*(const double rhs) // compound assignment (does not need to be a member,
  {                           // but often is, to modify the private members)
                /* addition of rhs to *this takes place here */
    CVec3d val;
    val.x = this->x * rhs;
    val.y = this->y * rhs;
    val.z = this->z * rhs;
    return val;
  }

  CVec3d operator+(const CVec3d& rhs) // compound assignment (does not need to be a member,
  {                           // but often is, to modify the private members)
                /* addition of rhs to *this takes place here */
    CVec3d val;
    val.x = this->x + rhs.x;
    val.y = this->y + rhs.y;
    val.z = this->z + rhs.z;
    return val;
  }

  CVec3d operator-(const CVec3d& rhs) // compound assignment (does not need to be a member,
  {                           // but often is, to modify the private members)
                /* addition of rhs to *this takes place here */
    CVec3d val;
    val.x = this->x - rhs.x;
    val.y = this->y - rhs.y;
    val.z = this->z - rhs.z;
    return val;
  }
};

class CVec2Img
{
public:
  int cols;
  int rows;
  CVec2f **arr;
  CVec2Img(){}
  CVec2Img(int w, int h)
  {
    cols = w;
    rows = h;

    arr = new CVec2f*[h];
    for (int i = 0; i < h; i++)
      arr[i] = new CVec2f[w];
  }

  ~CVec2Img() { }
  
  void Create(int w, int h)
  {
    cols = w;
    rows = h;

    arr = new CVec2f*[h];
    for (int i = 0; i < h; i++) {
      arr[i] = new CVec2f[w];
    }
    printf("Allocated");
  }
};

class FishOcam
{
public:
  const static int maxPolLength = 64;
  double pol[maxPolLength];    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;                // length of polynomial
  double invpol[maxPolLength]; // the coefficients of the inverse polynomial
  int length_invpol;             // length of inverse polynomial
  double xc;         // row coordinate of the center
  double yc;         // column coordinate of the center
  double c;          // affine parameter
  double d;          // affine parameter
  double e;          // affine parameter
  int width;         // image width
  int height;        // image height
  double invdet;

  FishOcam() {}
  
  void init(const std::string & calibFile)
  {
    FILE *f = fopen(calibFile.c_str(), "rb");

    const int maxBuf = 1024;
    char buf[maxBuf];

    //Read polynomial coefficients
    fgets(buf, maxBuf, f);
    fscanf(f, "\n");
    fscanf(f, "%d", &length_pol);
    for (int i = 0; i < length_pol; i++)
    {
      fscanf(f, " %lf", &pol[i]);
    }

    //Read inverse polynomial coefficients
    fscanf(f, "\n");
    fgets(buf, maxBuf, f);
    fscanf(f, "\n");
    fscanf(f, "%d", &length_invpol);
    for (int i = 0; i < length_invpol; i++)
    {
      fscanf(f, " %lf", &invpol[i]);
    }

    //Read center coordinates
    fscanf(f, "\n");
    fgets(buf, maxBuf, f);
    fscanf(f, "\n");
    fscanf(f, "%lf %lf\n", &xc, &yc);

    //Read affine coefficients
    fgets(buf, maxBuf, f);
    fscanf(f, "\n");
    fscanf(f, "%lf %lf %lf\n", &c, &d, &e);

    //Read image size
    fgets(buf, maxBuf, f);
    fscanf(f, "\n");
    fscanf(f, "%d %d", &height, &width);

    fclose(f);

    invdet = 1 / (c - d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file
  }

  CVec3d cam2world(const CVec2d & f)
  {
    double xp = invdet*((f.x - xc) - d*(f.y - yc));
    double yp = invdet*(-e*(f.x - xc) + c*(f.y - yc));

    double r = sqrt(xp*xp + yp*yp); //distance [pixels] of  the point from the image center
    double zp = pol[0];
    double r_i = 1;
    int i;

    for (i = 1; i < length_pol; i++)
    {
      r_i *= r;
      zp += r_i*pol[i];
    }

    //normalize to unit norm
    double invnorm = 1 / sqrt(xp*xp + yp*yp + zp*zp);

    CVec3d res;
    res.x = invnorm*xp;
    res.y = invnorm*yp;
    res.z = invnorm*zp;
    return res;
  }

  CVec2d world2cam(const CVec3d & p)
  {
    double norm = sqrt(p.x*p.x + p.y*p.y);
    double theta = atan(p.z / norm);

    CVec2d res;

    if (norm != 0)
    {
      double invnorm = 1 / norm;
      double t = theta;
      double rho = invpol[0];
      double t_i = 1;

      for (int i = 1; i < length_invpol; i++)
      {
        t_i *= t;
        rho += t_i*invpol[i];
      }

      double x = p.x*invnorm*rho;
      double y = p.y*invnorm*rho;

      res.x = x*c + y*d + xc;
      res.y = x*e + y + yc;
    }
    else
    {
      res.x = xc;
      res.y = yc;
    }
    return res;
  }

  void createPerspectiveWarp(CVec2Img & warp, int & hout, double & hfov, double & vfov, double & focal,
    const int win, const int hin, const int wout, const bool crop)
  {
    CVec3d wbase, wright, wup;

    if (crop)
    {
      CVec3d wcenter = cam2world(CVec2d(hin / 2.0, win / 2.0));

      wright = cam2world(CVec2d(hin / 2.0, win));
      wright = (wright / (wright*wcenter) - wcenter).Unit();

      wup = wcenter.Cross(wright);

      CVec3d w0 = cam2world(CVec2d(0, win / 2));
      w0 = w0 / (w0*wcenter);

      CVec3d w1 = cam2world(CVec2d(hin, win / 2));
      w1 = w1 / (w1*wcenter);

      CVec3d w2 = cam2world(CVec2d(hin / 2, 0));
      w2 = w2 / (w2*wcenter);

      CVec3d w3 = cam2world(CVec2d(hin / 2, win));
      w3 = w3 / (w3*wcenter);

      double extright = min(abs((w2 - wcenter)*wright), abs((w3 - wcenter)*wright));
      double extup = min(abs((w0 - wcenter)*wup), abs((w1 - wcenter)*wup));

      wbase = wcenter - wright*extright - wup*extup;
      wright = wright * (2.0*extright);
      wup = wup * (2.0*extup);

      hout = wout*extup / extright;
    }
    else
    {
      CVec3d wcenter = cam2world(CVec2d(hin / 2.0, win / 2.0));

      wright = cam2world(CVec2d(hin / 2.0, win));
      wright = (wright / (wright*wcenter) - wcenter).Unit();

      wup = wcenter.Cross(wright);

      CVec3d w0 = cam2world(CVec2d(0, 0));
      w0 = w0 / (w0*wcenter);

      CVec3d w1 = cam2world(CVec2d(0, win));
      w1 = w1 / (w1*wcenter);

      CVec3d w2 = cam2world(CVec2d(hin, win));
      w2 = w2 / (w2*wcenter);

      CVec3d w3 = cam2world(CVec2d(hin, 0));
      w3 = w3 / (w3*wcenter);

      double extright = max(max(abs((w0 - wcenter)*wright), abs((w1 - wcenter)*wright)), max(abs((w2 - wcenter)*wright), abs((w3 - wcenter)*wright)));
      double extup = max(max(abs((w0 - wcenter)*wup), abs((w1 - wcenter)*wup)), max(abs((w2 - wcenter)*wup), abs((w3 - wcenter)*wup)));

      wbase = wcenter - wright*extright - wup*extup;
      wright = wright * (2.0*extright);
      wup = wup * (2.0*extup);

      hout = wout*extup / extright;
    }

    warp.Create(wout, hout);

    for (int y = 0; y < hout; y++)
    {
      for (int x = 0; x < wout; x++)
      {
        CVec3d p = wbase;
        CVec3d temp1;
        temp1.x = (wright.x)*x / (wout - 1.0);
        temp1.y = (wright.y)*x / (wout - 1.0);
        temp1.z = (wright.z)*x / (wout - 1.0);

        CVec3d temp2;
        temp2.x = (wup.x)*y / (hout - 1.0);
        temp2.y = (wup.y)*y / (hout - 1.0);
        temp2.z = (wup.z)*y / (hout - 1.0);

        p.x += temp1.x + temp2.x;
        p.y += temp1.y + temp2.y;
        p.z += temp1.z + temp2.z;

        CVec2d f = world2cam(p);
        if (f.x < 0)
        {
          //std::cout << f.x << " fx was less\n";
          f.x = 0;
        }
        if (f.y < 0)
        {
          //std::cout << f.y << " fy was less\n";
          f.y = 0;
        }

        if (f.x > hin - 1)
        {
          //std::cout << f.x << " fx was more\n";
          f.x = hin - 1;
        }
        if (f.y > win - 1)
        {
          //std::cout << f.y << " fy was more\n";
          f.y = win -1;
        }


        if ((f.x < 0) || (f.y < 0)) {
    
          printf("yes\n");
        }
        warp.arr[y][x].x = f.y;
        warp.arr[y][x].y = f.x;
        assert(warp.arr[y][x].y <= hin -1 );
        assert(warp.arr[y][x].y >= 0);
        assert(warp.arr[y][x].x <= win -1);
        assert(warp.arr[y][x].x >= 0);
      }
    }

    hfov = acos((wbase + wup / 2).Unit() * (wbase + wup / 2 + wright).Unit());
    vfov = acos((wbase + wright / 2).Unit() * (wbase + wright / 2 + wup).Unit());
    focal = (wout > hout ?( 0.5 *wout )/ tan(hfov / 2) : (0.5 *hout) / tan(vfov / 2));
  }
};

int main()
{
  FishOcam f;
  string s = "wide/calib_results.txt";
  f.init(s);
  CVec2Img warp;
  int hout;
  const int wout = f.width;
  double hfov;
  double vfov;
  double focal;
  f.createPerspectiveWarp(warp, hout, hfov, vfov, focal, 1920, 1080, 1920, true);
  std::cout << "Focal is " << focal << "\n";

  string file_or_video_name = "wide/hostel.MP4";
  string out_video = "wide/hostel_out.avi";
  cv::VideoCapture inputVideo(file_or_video_name);
  VideoWriter outputVideo;
  int ex = static_cast<int>(inputVideo.get(CV_CAP_PROP_FOURCC));
  Size S = Size(wout, hout);
  
  int waitTime=0;

  if (!(inputVideo.isOpened())) {
    // check if we succeeded
    cerr << "Invalid video file" << endl;

    return -1;
  }

  outputVideo.open(out_video, ex, inputVideo.get(CV_CAP_PROP_FPS), S, true);


  if (!outputVideo.isOpened())
  {
    cout << "Could not open the output video for write: "  << endl;
    return -1;
  }

  int grabber_counter = 0;
  while (1) {
    grabber_counter+=1;
    Mat image;
    inputVideo.read(image);
    if (image.empty()) {
      if (grabber_counter >= 42)
        break;
      continue;
    }
    grabber_counter = 0;

    Mat image2(cv::Size(wout, hout), CV_8UC3);
    for (int i = 0; i < warp.rows; i++)
    {
      for (int j = 0; j < warp.cols; j++)
      {
        CVec2f tmp = warp.arr[i][j];
        image2.at<Vec3b>(i, j) = image.at<Vec3b>(tmp.y, tmp.x);
      }
    }

    outputVideo << image2;
  }
}
