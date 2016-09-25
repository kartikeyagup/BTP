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
  rm -r data2
  mkdir data2
  build/slam
fi
