if [ ! -d build ]; then
  echo "Creating build"
  mkdir build
fi

cd build
cmake ..
if [ $1 = "slam" ]; then 
  rm -rf slam 
elif [ $1 = "epipolar" ]; then
  rm -rf epipolar
else 
  rm -rf dense
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
  elif [ $1 = "dense" ]; then
    if [ -f dense ]; then
      cd ..
      build/dense -nvm_file="btp/outputVSFM_GB.nvm" -output_file="btp/newVSFM_GB.nvm" -output_ply="btp/newoutput.ply"
    fi
  else 
    echo "Invalid usage"
  fi
else 
  cd ..
fi
