
if [ ! -d build ]; then
  echo "Creating build"
  mkdir build
fi

cd build
cmake ..
rm -rf sensitivity
make
if [ -f sensitivity ]; then 
  cd ..
  build/sensitivity
fi 