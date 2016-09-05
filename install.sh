# KEEP UBUNTU OR DEBIAN UP TO DATE

sudo apt-get -E -y update
sudo apt-get -E -y upgrade
sudo apt-get -E -y dist-upgrade
sudo apt-get -E -y autoremove


# INSTALL THE DEPENDENCIES

# Build tools:
sudo apt-get -E install -y build-essential cmake

# GFlags
sudo apt-get -E install -y libgflags-dev

# GUI (if you want to use GTK instead of Qt, replace 'qt5-default' with 'libgtkglext1-dev' and remove '-DWITH_QT=ON' option in CMake):
sudo apt-get -E install -y qt5-default libvtk6-dev

# Media I/O:
sudo apt-get -E install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev

# Video I/O:
sudo apt-get -E install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev

# Parallelism and linear algebra libraries:
sudo apt-get -E install -y libtbb-dev libeigen3-dev

# Python:
sudo apt-get -E install -y python-dev python-tk python-numpy python3-dev python3-tk python3-numpy

# Java:
sudo apt-get -E install -y ant default-jdk

# Documentation:
sudo apt-get -E install -y doxygen

# OpenGL
sudo apt-get -E install -y freeglut3 freeglut3-dev

# INSTALL THE LIBRARY (YOU CAN CHANGE '3.1.0' FOR THE LAST STABLE VERSION)

sudo apt-get -E install -y unzip wget
wget https://github.com/Itseez/opencv/archive/3.1.0.zip
unzip 3.1.0.zip
mv opencv-3.1.0 OpenCV
cd OpenCV
mkdir build
cd build
cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DBUILD_EXAMPLES=ON ..
make -j4
sudo make install
sudo ldconfig
rm 3.1.0.zip
