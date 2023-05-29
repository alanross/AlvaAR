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
    message(STATUS "CMAKE_TOOLCHAIN_FILE not set. Using '~/Development/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake'")
    set(CMAKE_TOOLCHAIN_FILE "~/Development/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")
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
include_directories(${LIBS_BUILD_FOLDER}/opengv/include/)
include_directories(${LIBS_BUILD_FOLDER}/eigen/include/eigen3)
include_directories(${LIBS_BUILD_FOLDER}/Sophus/include/)
include_directories(${LIBS_BUILD_FOLDER}/ceres-solver/include/)
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