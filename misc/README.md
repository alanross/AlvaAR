# AlvaARを動かしてみる

## AlvaARって何
（[README.md](../READMS.md) より翻訳）
[AlvaAR](https://github.com/alanross/AlvaAR) は、Web ブラウザ上で WebAssembly で実行されるリアルタイム [SLAM（自己位置推定+環境地図作成）](https://ja.wikipedia.org/wiki/SLAM) 描写アルゴリズムです。 
[OV²SLAM](https://github.com/ov2slam/ov2slam) と [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) を大幅に修正したものです。SLAMは、ワールドトラッキングに焦点を当てたARアプリケーションの中核となる構成要素です。

## やりたいこと
- SLAMって何？
- とりあえず動かしてみたい
- なんとなくビルドもしたい
    - しばし WebAssembly に触ってないのでリハビリしたい気持ち

## やったこと
### ソースコード入手

[AlvaAR](https://github.com/alanross/AlvaAR) から入手。

```
$ git clone https://github.com/alanross/AlvaAR
```

### サンプルを動かそう

サンプルを動かすだけなら、[Run with http server](https://github.com/alanross/AlvaAR#run-with-http-server) の記載に従って行けばよい。

```
$ cd AlvaAR/examples
$ python -m http.server 8080
```

`http://localhost:8080/public/video.html` にアクセスすると、動かせる。

### ビルドしよう
#### tl;dr

[ビルド手順](../README.md#Build)にあるすべてを実行するスクリプトが [build.sh](../build.sh) にある。これで一発。

```
$ bash ./build.sh > build.log 2>&1
```

以降、ビルドを実行するにあたって、修正が必要だった箇所について説明する。

#### Setup Emscripten

このリポジトリのルートディレクトリにある docker compose 設定ファイルを使って、Emscripten 環境を起動する。

`ログ`

```
$ docker compose run emsdk_sandbox
Setting up EMSDK environment (suppress these messages with EMSDK_QUIET=1)
Setting environment variables:
PATH = /emsdk:/emsdk/upstream/emscripten:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
LICENSE                        emscripten-releases-tags.json  emsdk.py                       emsdk_manifest.json            hello.o                        legacy-emscripten-tags.txt     node/                          zips/
```

Emscripten を設定。

`ログ`

```
root@fe6a30ad5fa5:/sandbox# source /emsdk/emsdk_env.sh
Setting up EMSDK environment (suppress these messages with EMSDK_QUIET=1)
Adding directories to PATH:
PATH += /emsdk/node/16.20.0_64bit/bin

Setting environment variables:
PATH = /emsdk/node/16.20.0_64bit/bin:/emsdk:/emsdk/upstream/emscripten:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin
root@fe6a30ad5fa5:/sandbox# emcc -v
emcc (Emscripten gcc/clang-like replacement + linker emulating GNU ld) 3.1.42 (6ede0b8fc1c979bb206148804bfb48b472ccc3da)
clang version 17.0.0 (https://github.com/llvm/llvm-project f3b64887de61020c09404bfee97b2fadd30df10a)
Target: wasm32-unknown-emscripten
Thread model: posix
InstalledDir: /emsdk/upstream/bin
```

#### Build Dependencies

`AlvaAR/src/libs/build.sh` で、変数 `EMSCRIPTEN_DIR` を Emscripten がインストールされているパスに合わせて修正する。ここで使用している Docker image（emscripten/emsdk）では `/emsdk/upstream/emscripten` になる：

```bash
（中略）
# Ensure this is adjusted to your local emsdk path
# EMSCRIPTEN_DIR=~/Development/emsdk/upstream/emscripten
EMSCRIPTEN_DIR=/emsdk/upstream/emscripten
（以下省略）
```

また `sed` の構文がミスってる気がするので修正：

```bash
  # find $INSTALL_DIR/ceres-solver/include -type f -name '*.h' -exec sed -i '' s#glog/logging.h#ceres/internal/miniglog/glog/logging.h#g {} +
  find $INSTALL_DIR/ceres-solver/include -type f -name '*.h' -exec sed -i'' s#glog/logging.h#ceres/internal/miniglog/glog/logging.h#g {} +
```

さらに、Obindex2, iBoW-LCD は c++11 以外ではビルドできない（c++17 で削除された `std::unary_function` を使っている、など、、）ので、コンパイラオプションも修正。 `CMAKE_CXX_STANDARD=11` を指定する。

```bash
build_OBINDEX2() {
  rm -rf $INSTALL_DIR/obindex2/
  rm -rf $LIB_ROOT/obindex2/build
  mkdir -p $LIB_ROOT/obindex2/build

  cd $LIB_ROOT/obindex2/build
  emcmake cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=11 \
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
    -DCMAKE_CXX_STANDARD=11 \
    -DCMAKE_TOOLCHAIN_FILE=$EMSCRIPTEN_CMAKE_DIR \
    -DCMAKE_CXX_FLAGS="${BUILD_FLAGS} -s USE_BOOST_HEADERS=1" \
    -DCMAKE_C_FLAGS="${BUILD_FLAGS} -s USE_BOOST_HEADERS=1" \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/ibow_lcd/ \
    -DBUILD_SHARED_LIBS=OFF \
    -DOpenCV_DIR=$LIB_ROOT/opencv/build/
  emmake make -j install
}
```

<details>
<summary>AlvaAR/src/libs/build.sh</summary>

```bash
#!/bin/bash

# The lib directory
LIB_ROOT=$PWD

# Ensure this is adjusted to your local emsdk path
# EMSCRIPTEN_DIR=~/Development/emsdk/upstream/emscripten
EMSCRIPTEN_DIR=/emsdk/upstream/emscripten

# Emscripten cmake
EMSCRIPTEN_CMAKE_DIR=$EMSCRIPTEN_DIR/cmake/Modules/Platform/Emscripten.cmake

# Sets the compile flags. [SIMD, THREADS, DEFAULT]
BUILD_TYPE="DEFAULT"

if [ $BUILD_TYPE = "SIMD" ]; then
  echo "Compiling with SIMD enabled"
  INSTALL_DIR=$LIB_ROOT/build_simd/
  BUILD_FLAGS="-O3 -std=c++17 -msimd128";
  CONF_OPENCV="--simd";
elif [ $BUILD_TYPE = "THREADS" ]; then
  echo "Compiling with THREADS enabled"
  INSTALL_DIR=$LIB_ROOT/build_threads/
  BUILD_FLAGS="-O3 -std=c++17 -s USE_PTHREADS=1 -s PTHREAD_POOL_SIZE=4";
  CONF_OPENCV="--threads";
else
  echo "Compiling with DEFAULT settings"
  INSTALL_DIR=$LIB_ROOT/build/
  BUILD_FLAGS="-O3 -std=c++17";
  CONF_OPENCV="";
fi

build_OPENCV() {
  # To enable opencv_contrib-4.x modules add the following line to opencv/platforms/js/build_js.py -> def get_cmake_cmd(self):
  # "-DOPENCV_EXTRA_MODULES_PATH=[YOUR_PATH_TO_OPENCV_CONTRIB_DIR]/opencv_contrib-4.x/modules",
  # For more options look here: https://docs.opencv.org/4.x/d4/da1/tutorial_js_setup.html

  rm -rf $INSTALL_DIR/opencv/

  python $LIB_ROOT/opencv/platforms/js/build_js.py $INSTALL_DIR/opencv --build_wasm $CONF_OPENCV --emscripten_dir $EMSCRIPTEN_DIR
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
    -DCMAKE_CXX_STANDARD=11 \
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
    -DCMAKE_CXX_STANDARD=11 \
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
  find $INSTALL_DIR/ceres-solver/include -type f -name '*.h' -exec sed -i'' s#glog/logging.h#ceres/internal/miniglog/glog/logging.h#g {} +
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
    -DCMAKE_CXX_FLAGS="${BUILD_FLAGS} " \
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

```

</details>
 

次に、一部、ライブラリの元リポジトリにはあるが、ここにはないファイルを追加。
- [AlvaAR/src/libs/eigen/scripts/buildtests.in](https://github.com/libigl/eigen/blob/1f05f51517ec4fd91eed711e0f89e97a7c028c0e/scripts/buildtests.in)

<details>
<summary>buildtests.in</summary>

```bash
#!/bin/bash

if [[ $# != 1 || $1 == *help ]]
then
  echo "usage: $0 regexp"
  echo "  Builds tests matching the regexp."
  echo "  The EIGEN_MAKE_ARGS environment variable allows to pass args to 'make'."
  echo "    For example, to launch 5 concurrent builds, use EIGEN_MAKE_ARGS='-j5'"
  exit 0
fi

TESTSLIST="@EIGEN_TESTS_LIST@"
targets_to_make=`echo "$TESTSLIST" | egrep "$1" | xargs echo`

if [ -n "${EIGEN_MAKE_ARGS:+x}" ]
then
  @CMAKE_MAKE_PROGRAM@ $targets_to_make ${EIGEN_MAKE_ARGS}
else
  @CMAKE_MAKE_PROGRAM@ $targets_to_make @EIGEN_TEST_BUILD_FLAGS@
fi
exit $?

```

</details>


さらに、一部、パスやコンパイルオプションが自環境と合わない箇所があるので、`CMakeLists.txt` を修正。

<details>
<summary>AlvaAR/src/libs/opengv/CMakeLists.txt</summary>

```cmake
cmake_minimum_required(VERSION 3.1.3)
project(opengv VERSION 1.0 LANGUAGES CXX)

# set(CMAKE_TOOLCHAIN_FILE "~/Development/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")
set(CMAKE_TOOLCHAIN_FILE "/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")

:（省略）
IF(MSVC)
  add_compile_options(/wd4514 /wd4267 /bigobj)
  add_definitions(-D_USE_MATH_DEFINES)
ELSE()
  IF (CMAKE_SYSTEM_PROCESSOR MATCHES "(arm64)|(ARM64)|(aarch64)|(AARCH64)")
    add_definitions (-march=armv8-a)
  ELSEIF (CMAKE_SYSTEM_PROCESSOR MATCHES 
          "(arm)|(ARM)|(armhf)|(ARMHF)|(armel)|(ARMEL)")
    add_definitions (-march=armv7-a)
  ELSE ()
    # ↓削除
    # add_definitions (-march=native) #TODO use correct c++11 def once everybody has moved to gcc 4.7 # for now I even removed std=gnu++0x
  ENDIF()
:（省略）
```

</details>


<details>
<summary>AlvaAR/src/libs/obindex2/lib/CMakeLists.txt</summary>

```cmake
：（中略）
# Setting the flags for profiling information or not
if(CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "Setting Release options")
  # set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Setting Debug options")
  # set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O1 -pg -march=native")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O1 -pg")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O1 -pg -march=native")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O1 -pg")
endif()

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.
                         Please use a different C++ compiler.")
endif()

# Packages required to work with the library
set(OpenCV_DIR "/sandbox/alvaar/AlvaAR/src/libs/build/opencv/") # ビルドした OpenCV へのパス
find_package(OpenCV REQUIRED) # OpenCV
：（中略）
```

</details>

<details>
<summary>AlvaAR/src/libs/ibow_lcd/CMakeLists.txt</summary>

```cmake
：（中略）
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.
                         Please use a different C++ compiler.")
endif()

# Printing the compiling flags
set(OpenCV_DIR "/sandbox/alvaar/AlvaAR/src/libs/build/opencv/") # ビルドした OpenCV へのパス
message(STATUS "Compiler flags: ${CMAKE_CXX_FLAGS}")
：（中略）
```

</details>

修正したのちビルドスクリプトを実行する。

`ログ`

```
root@5c376668e09c:/sandbox# cd alvaar/AlvaAR/src/libs/
root@5c376668e09c:/sandbox/alvaar/AlvaAR/src/libs# sudo ln -s /usr/bin/python3 /usr/bin/python
root@5c376668e09c:/sandbox/alvaar/AlvaAR/src/libs# ./build.sh
（ログ省略）
Step 7/7 -------------------------------- Complete 
```

#### Build AlvaAR

依存ライブラリ群をビルドし終えたら、ArvaAR をビルドする。
先に、Emscripten のパスやインクルードパスがちょいちょい違っているので、自環境にあわせて `CMakeLists.txt` でインクルードパスを修正。

```
：（中略）

# Set Emscripten toolchain, if not defined
if (NOT CMAKE_TOOLCHAIN_FILE)
    # message(STATUS "CMAKE_TOOLCHAIN_FILE not set. Using '~/Development/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake'")
    message(STATUS "CMAKE_TOOLCHAIN_FILE not set. Using '/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake'")
    # set(CMAKE_TOOLCHAIN_FILE "~/Development/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")
    set(CMAKE_TOOLCHAIN_FILE "/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")
endif (NOT CMAKE_TOOLCHAIN_FILE)

：（中略）

include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/calib3d/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/core/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/features2d/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/flann/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/highgui/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/imgcodecs/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/imgproc/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/objdetect/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/video/include/)
include_directories(${LIBS_BUILD_FOLDER}/opencv/)
# include_directories(${LIBS_BUILD_FOLDER}/opengv/include/) # 存在しない
include_directories(${LIBS_BUILD_FOLDER}/../opengv/include/) # ☆add
# include_directories(${LIBS_BUILD_FOLDER}/eigen/include/eigen3)
include_directories(${LIBS_BUILD_FOLDER}/../eigen/) # add
include_directories(${LIBS_BUILD_FOLDER}/Sophus/include/)
# include_directories(${LIBS_BUILD_FOLDER}/ceres-solver/include/) # 存在しない
include_directories(${LIBS_BUILD_FOLDER}/../ceres-solver/internal/) # ☆add
include_directories(${LIBS_BUILD_FOLDER}/../ceres-solver/internal/ceres/miniglog/) # ☆add
include_directories(${LIBS_BUILD_FOLDER}/../ceres-solver/include/) # ☆add
# include_directories(${LIBS_BUILD_FOLDER}/../ceres-solver/config/) # ☆add
include_directories(${LIBS_BUILD_FOLDER}/../ceres-solver/build/config/) # ☆add
include_directories(/emsdk/upstream/emscripten/cache/sysroot/include/) # ☆add for glog
include_directories(src)
：（中略）
```


<details>
<summary>AlvaAR/src/slam/build/CMakeLists.txt</summary>

```cmakelists
cmake_minimum_required(VERSION 3.1)
set(PROJECT_NAME "alva_ar")
project(${PROJECT_NAME})

# Use C++ 17 by default
set(CMAKE_CXX_STANDARD 17)

# Ensure correct version of emscripten
if ("${EMSCRIPTEN_VERSION}" VERSION_GREATER_EQUAL 3.0.0)
    message(STATUS "Using Emscripten version ${EMSCRIPTEN_VERSION}")
else ()
    message(STATUS "Emscripten is not defined or it is older than version 3.0.0. Current version: '${EMSCRIPTEN_VERSION}'")
endif ()

# Set Emscripten toolchain, if not defined
if (NOT CMAKE_TOOLCHAIN_FILE)
    # message(STATUS "CMAKE_TOOLCHAIN_FILE not set. Using '~/Development/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake'")
    message(STATUS "CMAKE_TOOLCHAIN_FILE not set. Using '/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake'")
    # set(CMAKE_TOOLCHAIN_FILE "~/Development/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")
    set(CMAKE_TOOLCHAIN_FILE "/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")
endif (NOT CMAKE_TOOLCHAIN_FILE)

# Set "Release" as build type, if not defined
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "CMAKE_BUILD_TYPE not set. Using 'Release'")
    set(CMAKE_BUILD_TYPE Release)
endif (NOT CMAKE_BUILD_TYPE)

# Set dist directory, if not defined
if (NOT DIST_DIR)
    message(STATUS "DIST_DIR not set. Using './dist'")
    set(DIST_DIR ${CMAKE_BINARY_DIR}/../../../dist)
endif (NOT DIST_DIR)

set(LIBS_BUILD_FOLDER "../libs/build")

include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/calib3d/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/core/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/features2d/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/flann/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/highgui/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/imgcodecs/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/imgproc/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/objdetect/include/)
include_directories(${LIBS_BUILD_FOLDER}/../opencv/modules/video/include/)
include_directories(${LIBS_BUILD_FOLDER}/opencv/)
# include_directories(${LIBS_BUILD_FOLDER}/opengv/include/) # 存在しない
include_directories(${LIBS_BUILD_FOLDER}/../opengv/include/) # ☆add
# include_directories(${LIBS_BUILD_FOLDER}/eigen/include/eigen3)
include_directories(${LIBS_BUILD_FOLDER}/../eigen/) # add
include_directories(${LIBS_BUILD_FOLDER}/Sophus/include/)
# include_directories(${LIBS_BUILD_FOLDER}/ceres-solver/include/) # 存在しない
include_directories(${LIBS_BUILD_FOLDER}/../ceres-solver/internal/) # ☆add
include_directories(${LIBS_BUILD_FOLDER}/../ceres-solver/internal/ceres/miniglog/) # ☆add
include_directories(${LIBS_BUILD_FOLDER}/../ceres-solver/include/) # ☆add
# include_directories(${LIBS_BUILD_FOLDER}/../ceres-solver/config/) # ☆add
include_directories(${LIBS_BUILD_FOLDER}/../ceres-solver/build/config/) # ☆add
include_directories(/emsdk/upstream/emscripten/cache/sysroot/include/) # ☆add for glog
include_directories(src)

# Add the project source files
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})

# Point to third party libraries ( run src/libs/build.sh to compile these )
file(GLOB LIB_OPENCV
        "${LIBS_BUILD_FOLDER}/opencv/lib/libopencv_calib3d.a"
        "${LIBS_BUILD_FOLDER}/opencv/lib/libopencv_core.a"
        "${LIBS_BUILD_FOLDER}/opencv/lib/libopencv_features2d.a"
        "${LIBS_BUILD_FOLDER}/opencv/lib/libopencv_flann.a"
        "${LIBS_BUILD_FOLDER}/opencv/lib/libopencv_imgproc.a"
        "${LIBS_BUILD_FOLDER}/opencv/lib/libopencv_objdetect.a"
        "${LIBS_BUILD_FOLDER}/opencv/lib/libopencv_video.a")
file(GLOB LIB_CERES
        "${LIBS_BUILD_FOLDER}/ceres-solver/lib/libceres.a")
file(GLOB LIB_OPENGV
        "${LIBS_BUILD_FOLDER}/opengv/lib/libopengv.a")

# hide warnings
set_source_files_properties(${SRC_FILES} PROPERTIES COMPILE_FLAGS "-w")

# Link third party libraries
target_link_libraries(${PROJECT_NAME}
        ${LIB_OPENCV}
        ${LIB_CERES}
        ${LIB_OPENGV})

# Specify compile arguments
# https://emscripten.org/docs/optimizing/Optimizing-Code.html
# https://github.com/emscripten-core/emscripten/blob/main/src/settings.js
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Compile in DEBUG mode")
    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "\
      -s EXPORT_NAME='AlvaARWasm' \
      -s ENVIRONMENT=web,worker \
      -s INITIAL_MEMORY=256MB \
      -s ALLOW_MEMORY_GROWTH=1 \
      -s ALLOW_TABLE_GROWTH=1 \
      -s MODULARIZE=1 \
      -s EXPORT_ES6=1 \
      -s EXPORT_ALL=1 \
      -s SINGLE_FILE=1 \
      -s ASSERTIONS=1 \
      -s DYLINK_DEBUG=1 \
      -s EXPORTED_FUNCTIONS=['_malloc'] \
      -s ERROR_ON_UNDEFINED_SYMBOLS=1 \
      -s DISABLE_EXCEPTION_CATCHING=0 \
      -s USE_ZLIB=1 \
      -std=c++17 \
      -msimd128 \
      -O3 \
      -Oz \
      --extern-post-js ../../system.js \
      --no-entry \
      --no-check-features \
      --profiling-funcs \
      --bind")
else ()
    message(STATUS "Compile in RELEASE mode")
    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "\
      -s EXPORT_NAME='AlvaARWasm' \
      -s ENVIRONMENT=web,worker \
      -s INITIAL_MEMORY=256MB \
      -s ALLOW_MEMORY_GROWTH=1 \
      -s ALLOW_TABLE_GROWTH=1 \
      -s MODULARIZE=1 \
      -s EXPORT_ES6=1 \
      -s EXPORT_ALL=1 \
      -s SINGLE_FILE=1 \
      -s ASSERTIONS=0 \
      -s DYLINK_DEBUG=0 \
      -s EXPORTED_FUNCTIONS=['_malloc'] \
      -s ERROR_ON_UNDEFINED_SYMBOLS=0 \
      -s DISABLE_EXCEPTION_CATCHING=0 \
      -s USE_ZLIB=1 \
      -std=c++17 \
      -msimd128 \
      -O3 \
      -Oz \
      --extern-post-js ../../system.js \
      --no-entry \
      --no-check-features \
      --bind")
endif ()
unset(CMAKE_BUILD_TYPE CACHE)

# Install command will only be run if calling "emmake make install"
install(TARGETS ${PROJECT_NAME} DESTINATION ${DIST_DIR})
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_BINARY_DIR}/../../../examples/public/assets)
```

</details>

準備を終えたら、改めてビルドを実行する。

```
root@5c376668e09c:/sandbox/alvaar/AlvaAR/src/libs# cd ..
root@5c376668e09c:/sandbox/alvaar/AlvaAR/src# cd slam/
root@5c376668e09c:/sandbox/alvaar/AlvaAR/src/slam# mkdir build
root@5c376668e09c:/sandbox/alvaar/AlvaAR/src/slam# cd build/
root@5c376668e09c:/sandbox/alvaar/AlvaAR/src/slam/build# emcmake cmake ..
configure: cmake .. -DCMAKE_TOOLCHAIN_FILE=/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake -DCMAKE_CROSSCOMPILING_EMULATOR=/emsdk/node/16.20.0_64bit/bin/node
-- Using Emscripten version 3.1.42
-- CMAKE_BUILD_TYPE not set. Using 'Release'
-- DIST_DIR not set. Using './dist'
-- Compile in RELEASE mode
-- Configuring done
-- Generating done
-- Build files have been written to: /sandbox/alvaar/AlvaAR/src/slam/build
root@5c376668e09c:/sandbox/alvaar/AlvaAR/src/slam/build# emmake make install
make: make install
Consolidate compiler generated dependencies of target alva_ar
[  6%] Building CXX object CMakeFiles/alva_ar.dir/src/camera_calibration.cpp.o
[ 12%] Building CXX object CMakeFiles/alva_ar.dir/src/ceres_parametrization.cpp.o
[ 18%] Building CXX object CMakeFiles/alva_ar.dir/src/embind.cpp.o
[ 25%] Building CXX object CMakeFiles/alva_ar.dir/src/feature_extractor.cpp.o
[ 31%] Building CXX object CMakeFiles/alva_ar.dir/src/feature_tracker.cpp.o
[ 37%] Building CXX object CMakeFiles/alva_ar.dir/src/frame.cpp.o
[ 43%] Building CXX object CMakeFiles/alva_ar.dir/src/map_manager.cpp.o
[ 50%] Building CXX object CMakeFiles/alva_ar.dir/src/map_point.cpp.o
[ 56%] Building CXX object CMakeFiles/alva_ar.dir/src/mapper.cpp.o
[ 62%] Building CXX object CMakeFiles/alva_ar.dir/src/multi_view_geometry.cpp.o
[ 68%] Building CXX object CMakeFiles/alva_ar.dir/src/optimizer.cpp.o
[ 75%] Building CXX object CMakeFiles/alva_ar.dir/src/state.cpp.o
[ 81%] Building CXX object CMakeFiles/alva_ar.dir/src/system.cpp.o
[ 87%] Building CXX object CMakeFiles/alva_ar.dir/src/utils.cpp.o
[ 93%] Building CXX object CMakeFiles/alva_ar.dir/src/visual_frontend.cpp.o
[100%] Linking CXX executable alva_ar.js
[100%] Built target alva_ar
Install the project...
-- Install configuration: "Release"
-- Installing: /sandbox/alvaar/AlvaAR/src/slam/build/../../../dist/alva_ar.js
-- Installing: /sandbox/alvaar/AlvaAR/src/slam/build/../../../examples/public/assets/alva_ar.js
```

ビルドできたようです。さて、遊んでいきましょう。
