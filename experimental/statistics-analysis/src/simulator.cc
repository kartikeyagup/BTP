#include "simulator.h"

camera_frame present_frame;

void getImage(cv::Mat &image) {
  cv::Mat img(720, 1280, CV_8UC3);
  glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4);
  glPixelStorei(GL_PACK_ROW_LENGTH, img.step/img.elemSize());  
  glReadPixels(0, 0, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE, img.data);
  cv::flip(img, image, 0);
}

void display(void) {
  int const window_width  = glutGet(GLUT_WINDOW_WIDTH);
  int const window_height = glutGet(GLUT_WINDOW_HEIGHT);
  
  glClearColor(0, 0, 0.0, 1.0);
  glClearDepth(1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_COLOR_MATERIAL);
  glViewport(0, 0, window_width, window_height);
  glMatrixMode(GL_PROJECTION);
  
  if (false) {
    glLoadIdentity();
    glOrtho(0-window_width, window_width,0- window_height,window_height,-2000,2000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();  
  } else {
    glLoadIdentity();
    gluPerspective(120, 16.0/9.0, 100, 3000);
    gluLookAt(present_frame.position.x, 
      present_frame.position.y, 
      present_frame.position.z,
      present_frame.position.x - 100 * sin(present_frame.rotation*PI/180), 
      present_frame.position.y, 
      present_frame.position.z - 100 * cos(present_frame.rotation*PI/180),
      0,1,0);
  }

  for (int i=0; i<5; i++) {
    for (int j=0; j<5; j++) {
      glPushMatrix();
      glColor3f(getColorR(i,j),
         getColorG(i, j), 
         getColorB(i, j));
      glTranslatef(-200 + 100*i, -200 + 100*j, 0);
      glutSolidSphere(10, 31, 10);
      glPopMatrix();      
    }
  }

  glutSwapBuffers();  
  glutPostRedisplay();
}

void simulate_images(grid_params grid_description,
  motion_type motion,
  float angle,
  int num_images,
  float distance,
  camera_params intrinsics,
  cv::Point3f starting_point,
  std::vector<camera_frame> &output_frames) {

  int x = 1;
  glutInit(&x, NULL);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
  glutInitWindowSize(1280,720);
  glutCreateWindow("display");
  glutDisplayFunc(display);
  glEnable(GL_DEPTH_TEST);

  present_frame.position = starting_point; 
  present_frame.rotation = angle;
  present_frame.intrinsics = intrinsics;
  for (int i=0; i<num_images; i++) {
    std::cout << "Image " << i <<"\n";
    glutMainLoopEvent();
    getImage(present_frame.image);
    output_frames.push_back(present_frame);
    UpdatePosition(present_frame, distance, motion);
  }
};
