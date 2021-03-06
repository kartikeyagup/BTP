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
  build/sensitivity --motion=1 --dump_images=1 --verbose=1 --distance=0.5 --num_images=400
fi
