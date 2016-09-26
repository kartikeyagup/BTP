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
  rm -rf $2
  mkdir $2
  build/slam -dirname=$2 -video=$1
fi
