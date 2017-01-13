#include <GL/glut.h>
#include <GL/freeglut.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "nvmhelpers.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <gflags/gflags.h>
#include <unordered_map>
#include <IL/il.h>

using namespace std;
//sift id exist or not
nvm_file f1;
unordered_map<int,bool> points_covered;
vector< vector<Corr3D> > points_to_disp;
vector<string> file_to_disp;


int points_taken = 0;

// void initGL() 
// {
//    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
//    glClearDepth(1.0f);                   // Set background depth to farthest
//    glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
//    glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
//    glShadeModel(GL_SMOOTH);   // Enable smooth shading
//    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
// }

void getImage(cv::Mat &image) {
  cv::Mat img(720, 1280, CV_8UC3);
  glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4);
  glPixelStorei(GL_PACK_ROW_LENGTH, img.step/img.elemSize());  
  glReadPixels(0, 0, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE, img.data);
  cv::flip(img, image, 0);
}

void display(void) 
{
  int const window_width  = glutGet(GLUT_WINDOW_WIDTH);
  int const window_height = glutGet(GLUT_WINDOW_HEIGHT);
  
  glClearColor(236.0/256.0, 240.0/256.0, 241.0/256.0 ,0.0);
  glClearDepth(1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_COLOR_MATERIAL);
  glMatrixMode(GL_PROJECTION);
  glViewport(0, 0, window_width, window_height);
  // glViewport (0, window_height, window_width/2, window_height/2);
  if (false) 
  {
    glLoadIdentity();
    glOrtho(0-window_width, window_width,0- window_height,window_height,-2000,2000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();  
  } else {
    glLoadIdentity();
    gluPerspective(50, 16.0/9.0, 1, 30000);

    Eigen::Vector3f c = -f1.kf_data[points_taken].rotation.transpose()*f1.kf_data[points_taken].translation;
    c *= 10000;

    gluLookAt(c(0,0),
      c(1,0),
      c(2,0),
      c(0,0)+10*f1.kf_data[points_taken].rotation(2,0),
      c(1,0)+10*f1.kf_data[points_taken].rotation(2,1),
      c(2,0)+10*f1.kf_data[points_taken].rotation(2,2),
      0,-1,0);
    // gluLookAt(500,500,500,0,0,0,0,1,0);
  }
  for(int l = 0 ; l < points_taken ; l++)
  {
  	for(int i = 0 ; i < points_to_disp[l].size() ; i++)
  	{
      glPushMatrix();
      glColor3f(points_to_disp[l][i].color.x/256.0,
      			points_to_disp[l][i].color.y/256.0,
      			points_to_disp[l][i].color.z/256.0);
      // cout << "The points are as follows - " << points_to_disp[0][i].point_3d(0,0) << "  "
      // 										 << points_to_disp[0][i].point_3d(1,0) << "  " 
      // 										 << points_to_disp[0][i].point_3d(2,0) << endl;
      glTranslatef(points_to_disp[l][i].point_3d(0,0),
      points_to_disp[l][i].point_3d(1,0),
      points_to_disp[l][i].point_3d(2,0));
      // glColor3f(getColorR(i,j),
      //    getColorG(i, j), 
      //    getColorB(i, j));
      // glTranslatef(-200 + 100*i, -200 + 100*j, 0);

      // glViewport(0, 0, window_width/2, window_height);
      glutSolidSphere(1, 31, 10);
      glPopMatrix();      
  	}
  }

  glutSwapBuffers();  
  glutPostRedisplay();
}

int LoadImage(char *filename)
{
    ILboolean success; 
     ILuint image; 
 
    ilGenImages(1, &image); /* Generation of one image name */
     ilBindImage(image); /* Binding of image name */
     success = ilLoadImage(filename); /* Loading of the image filename by DevIL */
 
    if (success) /* If no error occured: */
    {
        /* Convert every colour component into unsigned byte. If your image contains alpha channel you can replace IL_RGB with IL_RGBA */
           success = ilConvertImage(IL_RGBA, IL_UNSIGNED_BYTE); 
 
        if (!success)
           {
                 return -1;
           }
    }
    else
        return -1;
 
    return image;
}

int main(int argc, char* argv[])
{
	f1 = nvm_file("../outputVSFM_GB.nvm");
	std::cerr << "Done reading 1\n";

	

	for(int i = 0 ; i < f1.kf_data.size() ; i++)
	{
		// cout << f1.kf_data[i].filename << endl;
		file_to_disp.push_back(f1.kf_data[i].filename);
		vector<Corr3D> new_element;
		for(int j = 0 ; j < f1.corr_data.size() ; j++)
		{
			if(points_covered.find(f1.corr_data[j].corr[0].siftid) == points_covered.end())
			{
				for(int k = 0 ; k < f1.corr_data[j].corr.size() ; k++)
				{
					if(f1.corr_data[j].corr[k].imgid == i)
					{
						new_element.push_back(f1.corr_data[j]);
						points_covered[f1.corr_data[j].corr[k].siftid] = true;
						break;
					}
				}		
			}
		}
		points_to_disp.push_back(new_element);
	}

	cout << "number of elements in points covered  " << points_covered.size() << endl;
	cout << "points to display size  " << points_to_disp.size() << endl;

	int x = 1; 
  	glutInit(&x, NULL);
  	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
 	glutInitWindowSize(1280,720);
  	glutCreateWindow("display");
  	
 	// glutInitWindowsPosition(70,70);
  	glutDisplayFunc(display);
  	glEnable(GL_DEPTH_TEST);
  	// glEnable(GL_DEPTH_TEST);
  	cv::Mat image;
    cv::Mat openglview;
  	for(int j = 0 ; j < f1.kf_data.size() ; j++)
  	{
  		glutMainLoopEvent();
      getImage(openglview);
  		points_taken += 1;
  		cout << "../data2/" + file_to_disp[j] << endl;
  		image = cv::imread("../data2/" + file_to_disp[j],CV_LOAD_IMAGE_COLOR);
  		// cv::imshow("im1", image);
      // cv::imshow("im2", openglview);
      cv::Size sz1 = image.size();
      cv::Size sz2 = openglview.size();
      cv::Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
      cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
      image.copyTo(left);
      cv::Mat right(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
      openglview.copyTo(right);
      cv::imshow("im3", im3);
  		cv::waitKey(10);
  	}


	return 0;
}
