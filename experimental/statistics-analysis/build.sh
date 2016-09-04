
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
  if [ -d tempdir ]; then
    rm -R tempdir
  fi
  build/sensitivity tempdir
fi 