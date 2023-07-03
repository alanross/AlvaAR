#!/bin/bash

# The lib directory
LIB_ROOT=$PWD

# Ensure this is adjusted to your local emsdk path
EMSCRIPTEN_DIR=~/Development/emsdk/upstream/emscripten

# Emscripten cmake
EMSCRIPTEN_CMAKE_DIR=$EMSCRIPTEN_DIR/cmake/Modules/Platform/Emscripten.cmake

# Sets the compile flags. [SIMD, THREADS, DEFAULT]
BUILD_TYPE="DEFAULT"

if [ $BUILD_TYPE = "SIMD" ]; then
  echo "Compiling with SIMD enabled"
  INSTALL_DIR=$LIB_ROOT/build_simd
  BUILD_FLAGS="-O3 -std=c++17 -msimd128";
  CONF_OPENCV="--simd";
elif [ $BUILD_TYPE = "THREADS" ]; then
  echo "Compiling with THREADS enabled"
  INSTALL_DIR=$LIB_ROOT/build_threads
  BUILD_FLAGS="-O3 -std=c++17 -s USE_PTHREADS=1 -s PTHREAD_POOL_SIZE=4";
  CONF_OPENCV="--threads";
else
  echo "Compiling with DEFAULT settings"
  INSTALL_DIR=$LIB_ROOT/build
  BUILD_FLAGS="-O3 -std=c++17";
  CONF_OPENCV="";
fi

build_OPENCV() {
  # To enable opencv_contrib-4.x modules add the following line to opencv/platforms/js/build_js.py -> def get_cmake_cmd(self):
  # "-DOPENCV_EXTRA_MODULES_PATH=[YOUR_PATH_TO_OPENCV_CONTRIB_DIR]/opencv_contrib-4.x/modules",
  # For more options look here: https://docs.opencv.org/4.x/d4/da1/tutorial_js_setup.html

  rm -rf $INSTALL_DIR/opencv/
  rm -rf $LIB_ROOT/opencv/build

  python $LIB_ROOT/opencv/platforms/js/build_js.py $LIB_ROOT/opencv/build --build_wasm $CONF_OPENCV --emscripten_dir $EMSCRIPTEN_DIR
  cp -r $LIB_ROOT/opencv/build $INSTALL_DIR/opencv/
}

build_EIGEN() {
  rm -rf $INSTALL_DIR/eigen/
  rm -rf $LIB_ROOT/eigen/build
  mkdir -p $LIB_ROOT/eigen/build

  cd $LIB_ROOT/eigen/build
  emcmake cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_TOOLCHAIN_FILE=$EMSCRIPTEN_CMAKE_DIR \
    -DCMAKE_CXX_FLAGS="${BUILD_FLAGS}" \
    -DCMAKE_C_FLAGS="${BUILD_FLAGS}" \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/eigen/ \
    -DBUILD_SHARED_LIBS=OFF
  emmake make -j install
}

build_OBINDEX2() {
  rm -rf $INSTALL_DIR/obindex2/
  rm -rf $LIB_ROOT/obindex2/build
  mkdir -p $LIB_ROOT/obindex2/build

  cd $LIB_ROOT/obindex2/build
  emcmake cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_TOOLCHAIN_FILE=$EMSCRIPTEN_CMAKE_DIR \
    -DCMAKE_CXX_FLAGS="${BUILD_FLAGS} -s USE_BOOST_HEADERS=1" \
    -DCMAKE_C_FLAGS="${BUILD_FLAGS} -s USE_BOOST_HEADERS=1" \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/obindex2/ \
    -DBUILD_SHARED_LIBS=OFF \
    -DOpenCV_DIR=$LIB_ROOT/opencv/build/
  emmake make -j install
}

build_IBOW_LCD(){
  rm -rf $INSTALL_DIR/ibow_lcd/
  rm -rf $LIB_ROOT/ibow_lcd/build
  mkdir -p $LIB_ROOT/ibow_lcd/build

  cd $LIB_ROOT/ibow_lcd/build
  emcmake cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_TOOLCHAIN_FILE=$EMSCRIPTEN_CMAKE_DIR \
    -DCMAKE_CXX_FLAGS="${BUILD_FLAGS} -s USE_BOOST_HEADERS=1" \
    -DCMAKE_C_FLAGS="${BUILD_FLAGS} -s USE_BOOST_HEADERS=1" \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/ibow_lcd/ \
    -DBUILD_SHARED_LIBS=OFF \
    -DOpenCV_DIR=$LIB_ROOT/opencv/build/
  emmake make -j install
}

build_SOPHUS(){
  rm -rf $INSTALL_DIR/Sophus/
  rm -rf $LIB_ROOT/Sophus/build
  mkdir -p $LIB_ROOT/Sophus/build

  cd $LIB_ROOT/Sophus/build
  emcmake cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_TOOLCHAIN_FILE=$EMSCRIPTEN_CMAKE_DIR \
    -DCMAKE_CXX_FLAGS="${BUILD_FLAGS}" \
    -DCMAKE_C_FLAGS="${BUILD_FLAGS}" \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/Sophus/ \
    -DBUILD_SHARED_LIBS=OFF \
    -DEIGEN3_INCLUDE_DIR=$LIB_ROOT/eigen/
  emmake make -j install
}

build_CERES(){
  # When done compiling a string replace is called on all files in ceres-solver/install/include
  # to replace "glog/logging.h" with "ceres/internal/miniglog/glog/logging.h"

  rm -rf $INSTALL_DIR/ceres-solver/
  rm -rf $LIB_ROOT/ceres-solver/build
  mkdir $LIB_ROOT/ceres-solver/build

  cd $LIB_ROOT/ceres-solver/build
  emcmake cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_TOOLCHAIN_FILE=$EMSCRIPTEN_CMAKE_DIR \
    -DCMAKE_CXX_FLAGS="${BUILD_FLAGS} -march=native" \
    -DCMAKE_C_FLAGS="${BUILD_FLAGS} -march=native" \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/ceres-solver/ \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_EXAMPLES:BOOL=0 \
    -DBUILD_TESTING:BOOL=0 \
    -DEIGENSPARSE:BOOL=1 \
    -DCERES_THREADING_MODEL="NO_THREADS" \
    -DMINIGLOG:BOOL=1 \
    -DEigen3_DIR=$LIB_ROOT/eigen/build/
  emmake make -j install
  find $INSTALL_DIR/ceres-solver/include -type f -name '*.h' -exec sed -i '' s#glog/logging.h#ceres/internal/miniglog/glog/logging.h#g {} +
}

build_OPENGV(){
  rm -rf $INSTALL_DIR/opengv/
  rm -rf $LIB_ROOT/opengv/build
  mkdir $LIB_ROOT/opengv/build

  cd $LIB_ROOT/opengv/build
  emcmake cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_TOOLCHAIN_FILE=$EMSCRIPTEN_CMAKE_DIR \
    -DCMAKE_CXX_FLAGS="${BUILD_FLAGS}" \
    -DCMAKE_C_FLAGS="${BUILD_FLAGS}" \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/opengv/ \
    -DBUILD_SHARED_LIBS=OFF \
    -DEIGEN_INCLUDE_DIR=$LIB_ROOT/eigen/
  emmake make -j install
}

build() {
    array=($@)
    length=${#array[@]}

    BL='\033[1;34m'
    NC='\033[0m'

    for (( i=0; i<length; i++ ));
    do
      echo -e "${BL}Step $(($i+1))/$length -------------------------------- Start building: ${array[$i]} ${NC}"
      build_${array[$i]}
      echo -e "${BL}Step $(($i+1))/$length -------------------------------- Complete ${NC}\n\n"
    done
}

libsToBuild=( "EIGEN" "OPENCV" "OBINDEX2" "IBOW_LCD" "SOPHUS" "CERES" "OPENGV" )

build ${libsToBuild[@]}