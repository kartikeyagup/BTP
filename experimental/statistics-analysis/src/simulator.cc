#include "simulator.h"

camera_frame present_frame;

void getImage(cv::Mat &image) {
  cv::Mat img(720, 1280, CV_8UC3);
  glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4);
  glPixelStorei(GL_PACK_ROW_LENGTH, img.step / img.elemSize());
  glReadPixels(0, 0, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE, img.data);
  cv::flip(img, image, 0);
}

/* ascii code for the escape key */
#define ESCAPE 27

/* The number of our GLUT window */
int window;

/* storage for one texture  */
unsigned int texture[2];

/* Image type - contains height, width, and data */
struct Image {
  unsigned long sizeX;
  unsigned long sizeY;
  char *data;
};
typedef struct Image Image;

// quick and dirty bitmap loader...for 24 bit bitmaps with 1 plane only.
// See http://www.dcs.ed.ac.uk/~mxr/gfx/2d/BMP.txt for more info.
int ImageLoad(char *filename, Image *image) {
  FILE *file;
  unsigned long size;         // size of the image in bytes.
  unsigned long i;            // standard counter.
  unsigned short int planes;  // number of planes in image (must be 1)
  unsigned short int bpp;     // number of bits per pixel (must be 24)
  char temp;                  // temporary color storage for bgr-rgb conversion.

  // make sure the file is there.
  if ((file = fopen(filename, "rb")) == NULL) {
    printf("File Not Found : %s\n", filename);
    return 0;
  }

  // seek through the bmp header, up to the width/height:
  fseek(file, 18, SEEK_CUR);

  // read the width
  if ((i = fread(&image->sizeX, 4, 1, file)) != 1) {
    printf("Error reading width from %s.\n", filename);
    return 0;
  }
  printf("Width of %s: %lu\n", filename, image->sizeX);

  // read the height
  if ((i = fread(&image->sizeY, 4, 1, file)) != 1) {
    printf("Error reading height from %s.\n", filename);
    return 0;
  }
  printf("Height of %s: %lu\n", filename, image->sizeY);

  // calculate the size (assuming 24 bits or 3 bytes per pixel).
  size = image->sizeX * image->sizeY * 3;

  // read the planes
  if ((fread(&planes, 2, 1, file)) != 1) {
    printf("Error reading planes from %s.\n", filename);
    return 0;
  }
  if (planes != 1) {
    printf("Planes from %s is not 1: %u\n", filename, planes);
    return 0;
  }

  // read the bpp
  if ((i = fread(&bpp, 2, 1, file)) != 1) {
    printf("Error reading bpp from %s.\n", filename);
    return 0;
  }
  if (bpp != 24) {
    printf("Bpp from %s is not 24: %u\n", filename, bpp);
    return 0;
  }

  // seek past the rest of the bitmap header.
  fseek(file, 24, SEEK_CUR);

  // read the data.
  image->data = (char *)malloc(size);
  if (image->data == NULL) {
    printf("Error allocating memory for color-corrected image data");
    return 0;
  }

  if ((i = fread(image->data, size, 1, file)) != 1) {
    printf("Error reading image data from %s.\n", filename);
    return 0;
  }

  for (i = 0; i < size; i += 3) {  // reverse all of the colors. (bgr -> rgb)
    temp = image->data[i];
    image->data[i] = image->data[i + 2];
    image->data[i + 2] = temp;
  }
  fclose(file);
  // we're done.
  return 1;
}

// Load Bitmaps And Convert To Textures

void LoadGLTextures() {
  // Load Texture
  Image *image1, *image2, *image3, *image4, *image5, *image6;

  // allocate space for texture
  image1 = (Image *)malloc(sizeof(Image));
  image2 = (Image *)malloc(sizeof(Image));

  if (image1 == NULL) {
    printf("Error allocating space for image");
    exit(0);
  }
  if (image2 == NULL) {
    printf("Error allocating space for image");
    exit(0);
  }

  image3 = (Image *)malloc(sizeof(Image));
  image4 = (Image *)malloc(sizeof(Image));
  image5 = (Image *)malloc(sizeof(Image));
  image6 = (Image *)malloc(sizeof(Image));
  if (image3 == NULL) {
    printf("Error allocating space for image");
    exit(0);
  }
  if (image4 == NULL) {
    printf("Error allocating space for image");
    exit(0);
  }
  if (image5 == NULL) {
    printf("Error allocating space for image");
    exit(0);
  }
  if (image6 == NULL) {
    printf("Error allocating space for image");
    exit(0);
  }

  if (!ImageLoad("i1.bmp", image1)) {
    exit(1);
  }
  if (!ImageLoad("i2.bmp", image2)) {
    exit(1);
  }
  // Create Texture
  glGenTextures(1, &texture[0]);
  glBindTexture(GL_TEXTURE_2D, texture[0]);  // 2d texture (x and y size)
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                  GL_LINEAR);  // scale linearly when image bigger than texture
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                  GL_LINEAR);  // scale linearly when image smalled than texture
  // 2d texture, level of detail 0 (normal), 3 components (red, green, blue), x
  // size from image, y size from image,
  // border 0 (normal), rgb color data, unsigned byte data, and finally the data
  // itself.
  glTexImage2D(GL_TEXTURE_2D, 0, 3, image1->sizeX, image1->sizeY, 0, GL_RGB,
               GL_UNSIGNED_BYTE, image1->data);
  // Create Texture
  glGenTextures(1, &texture[1]);
  glBindTexture(GL_TEXTURE_2D, texture[1]);  // 2d texture (x and y size)
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                  GL_LINEAR);  // scale linearly when image bigger than texture
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                  GL_LINEAR);  // scale linearly when image smalled than texture
  // 2d texture, level of detail 0 (normal), 3 components (red, green, blue), x
  // size from image, y size from image,
  // border 0 (normal), rgb color data, unsigned byte data, and finally the data
  // itself.
  glTexImage2D(GL_TEXTURE_2D, 0, 3, image2->sizeX, image2->sizeY, 0, GL_RGB,
               GL_UNSIGNED_BYTE, image2->data);
};

// void DrawGLScene()
void drawCube(int x, int y, int z, int size, int i) {
  // glTranslatef(0.0f,0.0f,500.0f);              // move 5 units into the
  // screen.
  glColor3f(255, 255, 255);
  glBindTexture(GL_TEXTURE_2D, texture[i]);  // choose the texture to use.
  glBegin(GL_QUADS);                         // begin drawing a cube
  // Front Face (note that the texture's corners have to match the quad's
  // corners)
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(x - size / 2, y - size / 2,
             z + size / 2);  // Bottom Left Of The Texture and Quad
  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(x + size / 2, y - size / 2,
             z + size / 2);  // Bottom Right Of The Texture and Quad
  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(x + size / 2, y + size / 2,
             z + size / 2);  // Top Right Of The Texture and Quad
  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(x - size / 2, y + size / 2,
             z + size / 2);  // Top Left Of The Texture and Quad
  glEnd();                   // done with the polygon.
  // since this is double buffered, swap the buffers to display what just got
  // drawn.
  glutSwapBuffers();
}

void display(void) {
  int const window_width = glutGet(GLUT_WINDOW_WIDTH);
  int const window_height = glutGet(GLUT_WINDOW_HEIGHT);
  glClearColor(0.9, 0.5, 0.15, 1.0);
  glClearDepth(1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_COLOR_MATERIAL);
  glViewport(0, 0, window_width, window_height);
  glMatrixMode(GL_PROJECTION);

  if (false) {
    glLoadIdentity();
    glOrtho(0 - window_width, window_width, 0 - window_height, window_height,
            -2000, 2000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  } else {
    glLoadIdentity();
    gluPerspective(120, 16.0 / 9.0, 1, 3000);
    gluLookAt(
        present_frame.position.x, present_frame.position.y,
        present_frame.position.z,
        present_frame.position.x - 100 * sin(present_frame.rotation * PI / 180),
        present_frame.position.y,
        present_frame.position.z - 100 * cos(present_frame.rotation * PI / 180),
        0, 1, 0);
    /* // For fixed image
    gluLookAt(-700,-500,500,
              0,0,0,0,1,0); */
  }

  // for (int i=0; i<5; i++) {
  //   for (int j=0; j<5; j++) {
  //     glPushMatrix();
  //     glColor3f(getColorR(i,j),
  //        getColorG(i, j),
  //        getColorB(i, j));
  //     glTranslatef(-200 + 100*i, -200 + 100*j, 0);
  //     glutSolidSphere(10, 31, 10);
  //     glPopMatrix();
  //   }
  // }

  drawCube(0, 0, -100, 256, 0);
  drawCube(700, 500, 0, 128, 1);
  drawCube(0, 500, -50, 256, 0);
  drawCube(-400, 50, 100, 128, 1);
  drawCube(500, -100, -200, 512, 0);
  drawCube(-200, -200, 200, 128, 1);
  drawCube(-600, 200, 50, 256, 0);
  glutSwapBuffers();
  glutPostRedisplay();
}

void simulate_images(grid_params grid_description, motion_type motion,
                     float angle, int num_images, float distance,
                     camera_params intrinsics, cv::Point3f starting_point,
                     std::vector<camera_frame> &output_frames) {
  int x = 1;
  glutInit(&x, NULL);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
  glutInitWindowSize(1280, 720);
  glutCreateWindow("display");
  glutDisplayFunc(display);
  glEnable(GL_DEPTH_TEST);
  LoadGLTextures();  // Load The Texture(s)
  std::cout << "Loaded all\n";
  glEnable(GL_TEXTURE_2D);  // Enable Texture Mapping

  present_frame.position = starting_point;
  present_frame.rotation = angle;
  present_frame.intrinsics = intrinsics;
  float t = 0.0;
  for (int i = 0; i < num_images / 2; i++) {
    glutMainLoopEvent();
    getImage(present_frame.image);
    output_frames.push_back(present_frame);
    UpdatePosition(present_frame, distance, motion);
  }
  for (int i = 0; i < num_images / 2; i++) {
    glutMainLoopEvent();
    getImage(present_frame.image);
    output_frames.push_back(present_frame);
    t = (i * 1.0) / (num_images * 7.0);
    UpdatePosition1(present_frame, t);
  }
  for (int i = 0; i < num_images / 2; i++) {
    glutMainLoopEvent();
    getImage(present_frame.image);
    output_frames.push_back(present_frame);
    t = 1.0 / 14.0 - (i * 1.0) / (num_images * 7.0);
    UpdatePosition1(present_frame, t);
    // UpdatePosition(present_frame, -1.0*distance, motion);
  }
  for (int i = 0; i < num_images / 2; i++) {
    glutMainLoopEvent();
    getImage(present_frame.image);
    output_frames.push_back(present_frame);
    UpdatePosition(present_frame, -1.0 * distance, motion);
  }
};
