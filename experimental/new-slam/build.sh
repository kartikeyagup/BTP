if [ ! -d build ]; then
  echo "Creating build"
  mkdir build
fi

cd build
cmake ..
rm -rf slam
make
if [ -f slam ]; then 
  cd ..
  build/slam
fi
