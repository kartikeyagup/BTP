if [ ! -d build ]; then
  echo "Creating build"
  mkdir build
fi

cd build
cmake ..
if [ $1 = "slam" ]; then 
  rm -rf slam 
else
  rm -rf epipolar
fi
make -j4

if [ "$#" -ne 0 ]; then
  if [ $1 = "slam" ]; then
    if [ -f slam ]; then 
      cd ..
      if [ -d $3 ]; then
        rm -rf $3
        mkdir $3
        echo "Starting processing"
        build/slam -dirname=$3 -video=$2
      fi
    fi
  elif [ $1 = "epipolar" ]; then  
    if [ -f  epipolar ]; then
      cd ..
      build/epipolar
    fi
  else 
    echo "Invalid usage"
  fi
else 
  cd ..
fi
