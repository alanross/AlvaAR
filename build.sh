#! /usr/bin/env bash

export EMSDK_DIR="/emsdk"
export EMSCRIPTEN_DIR="${EMSDK_DIR}/upstream/emscripten"
export ALVAAR_ROOT_DIR="$(cd $(dirname $0); pwd)"
export ALVAAR_LIB_DIR="${ALVAAR_ROOT_DIR}/src/libs"
export ALVAAR_SLAM_DIR="${ALVAAR_ROOT_DIR}/src/slam"
export ALVAAR_SLAM_BUILD_DIR="${ALVAAR_SLAM_DIR}/build"

############################################################################## 
# Setup Emscripten
##############################################################################

source ${EMSDK_DIR}/emsdk_env.sh


##############################################################################
# Run Dependencies build
##############################################################################

# cd ${ALVAAR_LIB_DIR}
# bash ./build.sh > build.log 2>&1


##############################################################################
# Run build
##############################################################################

if [[ ! -e ${ALVAAR_SLAM_BUILD_DIR} ]]; then
	mkdir ${ALVAAR_SLAM_BUILD_DIR}
fi;

cd ${ALVAAR_SLAM_BUILD_DIR}
emcmake cmake .. > emcmake.cmake.log 2>&1
emmake make install > emcmake.make.install.log 2>&1

