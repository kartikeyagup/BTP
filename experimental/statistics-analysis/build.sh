if [ ! -d build ]; then
  echo "Creating build"
  mkdir build
fi

cd build
cmake ..
make
cd ..
build/sensitivity
