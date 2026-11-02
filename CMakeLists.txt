cmake_minimum_required(VERSION 3.11)

if(NOT CMAKE_CUDA_COMPILER)
  if(DEFINED ENV{CUDA_HOME})
    set(CMAKE_CUDA_COMPILER "$ENV{CUDA_HOME}/bin/nvcc" CACHE FILEPATH "CUDA compiler")
  elseif(EXISTS "/usr/local/cuda-12.9/bin/nvcc")
    set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.9/bin/nvcc" CACHE FILEPATH "CUDA compiler")
  elseif(EXISTS "/usr/local/cuda/bin/nvcc")
    set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "CUDA compiler")
  endif()
endif()

if(DEFINED LIBTORCH_ROOT OR DEFINED ENV{LIBTORCH_ROOT} OR CMAKE_CUDA_COMPILER)
  project(ORB_SLAM3 LANGUAGES CXX C CUDA)
else()
  project(ORB_SLAM3 LANGUAGES CXX C)
endif()

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3") #其中${CMAKE_C_FLAGS}表示先前在CMake中定义的C语言编译器标志，而-Wall -O3是额外添加到这个变量中的编译器标志。
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w   -O3")
set(CMAKE_CXX_FLAGS "-std=c++14 -w -fopenmp -march=native -pthread")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -O3")
set(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS} -g")
# Check C++11 or C++0x support

# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#    add_definitions(-DCOMPILEDWITHC11)
#    message(STATUS "Using flag -std=c++11.")
# elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#    add_definitions(-DCOMPILEDWITHC0X)
#    message(STATUS "Using flag -std=c++0x.")
# else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)

include(${PROJECT_SOURCE_DIR}/cmake/modules/libtorch.cmake)

# 要保证整个工程的opencv版本一致，包括dbow，源码以及ros相关的
# 3 4 都可以正常运行//必须要opencv3.4.13,要不然会在加载词袋处卡住

find_package(OpenCV 3.4.13 REQUIRED)
include(${PROJECT_SOURCE_DIR}/cmake/modules/light_glue_cpp.cmake)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.2 not found.")
   endif()


MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})


find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)
# find_package(realsense2)

# ONNX Runtime GPU 1.16.3: extract tarball next to the repo or pass -DONNXRUNTIME_ROOT=...
set(ONNXRUNTIME_ROOT "${PROJECT_SOURCE_DIR}/onnxruntime-linux-x64-gpu-1.16.3" CACHE PATH "Root of extracted onnxruntime-linux-x64-gpu-1.16.3 (must contain include/ and lib/)")
if(NOT EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
  message(FATAL_ERROR "ONNX Runtime not found at ONNXRUNTIME_ROOT=${ONNXRUNTIME_ROOT}\n"
    "  Expected file: ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so\n"
    "  Extract onnxruntime-linux-x64-gpu-1.16.3 into the repo root, or run cmake with:\n"
    "  cmake -DONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-gpu-1.16.3 ..")
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${ONNXRUNTIME_ROOT}/include
${CUDA_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${YAML_CPP_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${GFLAGS_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED 
src/Random.cpp
src/Timestamp.cpp
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PortalObservationStore.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
src/Extractors/SPextractor.cc
src/Extractors/AlikedExtractor.cc
src/Extractors/superpoint_onnx.cc
src/Matchers/SPmatcher.cc
src/Matchers/LightGlueMrNerfMatcher.cc
src/Matchers/lightglue_onnx.cpp
src/Matchers/transform.cpp
include/Timestamp.h
include/Random.h
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/PortalObservationStore.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Extractors/SPextractor.h
include/Extractors/AlikedExtractor.h
include/Extractors/superpoint_onnx.h
include/Matchers/SPmatcher.h
include/Matchers/LightGlueMrNerfMatcher.h
include/Matchers/lightglue_onnx.h
include/Matchers/Configuration.h
include/Matchers/transform.h
include/Settings.h)

add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
${CUDA_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW3/lib/libDBoW3.so
g2o
-lboost_serialization
-lcrypto
-Wl,--no-as-needed
${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so
-Wl,--as-needed
)

if(AUKI_LIGHTGLUE_AVAILABLE)
  target_compile_definitions(${PROJECT_NAME} PRIVATE AUKI_LIGHTGLUE_AVAILABLE)
  target_include_directories(${PROJECT_NAME} PRIVATE "${LIGHT_GLUE_CPP_DIR}/include")
  target_link_libraries(${PROJECT_NAME}
    auki_lightglue_lib
    ${TORCH_LIBRARIES})
  set(_auki_wrapper_sources
    src/Extractors/AlikedExtractor.cc
    src/Matchers/LightGlueMrNerfMatcher.cc)
  # Global CMAKE_CXX_FLAGS uses -std=c++14; override on wrapper TUs for LibTorch.
  set_source_files_properties(${_auki_wrapper_sources} PROPERTIES
    COMPILE_OPTIONS "-std=gnu++20"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON)
  target_compile_options(${PROJECT_NAME} PRIVATE ${TORCH_CXX_FLAGS})
endif()


find_package(ZXing QUIET CONFIG)
set(SLAM_SERVER_ZXING_SYSTEM_HEADERS OFF)
if(ZXing_FOUND)
  set(SLAM_SERVER_ZXING_SYSTEM_HEADERS ON)
else()
  include(FetchContent)
  set(BUILD_WRITERS OFF CACHE BOOL "Build ZXing writers" FORCE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "Build ZXing examples" FORCE)
  set(BUILD_BLACKBOX_TESTS OFF CACHE BOOL "Build ZXing blackbox tests" FORCE)
  set(BUILD_UNIT_TESTS OFF CACHE BOOL "Build ZXing unit tests" FORCE)
  set(BUILD_PYTHON_MODULE OFF CACHE BOOL "Build ZXing Python module" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build ZXing as a static library" FORCE)

  FetchContent_Declare(
    zxing_cpp
    GIT_REPOSITORY https://github.com/zxing-cpp/zxing-cpp.git
    GIT_TAG v1.1.0
    GIT_SHALLOW TRUE
  )
  if(COMMAND FetchContent_MakeAvailable)
    FetchContent_MakeAvailable(zxing_cpp)
  else()
    FetchContent_GetProperties(zxing_cpp)
    if(NOT zxing_cpp_POPULATED)
      FetchContent_Populate(zxing_cpp)
      add_subdirectory(${zxing_cpp_SOURCE_DIR} ${zxing_cpp_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
  endif()
  if(TARGET ZXing AND NOT MSVC)
    # ZXing-C++ v1.1.0 is C++14-compatible but misses <limits> in a few headers on newer GCC.
    target_compile_options(ZXing PRIVATE -include limits)
  endif()
endif()
if(NOT TARGET ZXing::ZXing)
  message(FATAL_ERROR "ZXing::ZXing target not available")
endif()

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBHEIF QUIET libheif)
endif()

# Build examples



#Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
        Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
        Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
        Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_tum_vi
        Examples/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${PROJECT_NAME})

find_package(ZLIB REQUIRED)

add_executable(slam_server
        Examples/Monocular/slam_server.cc
        Examples/Monocular/prior_pack_loader.cc
        Examples/Monocular/overlay_pose_graph.cc
        Examples/Monocular/prior_soft_edges.cc
        Thirdparty/cnpy/cnpy.cpp)
target_include_directories(slam_server PRIVATE Thirdparty/cnpy)
target_link_libraries(slam_server ${PROJECT_NAME} ZXing::ZXing ZLIB::ZLIB)
if(SLAM_SERVER_ZXING_SYSTEM_HEADERS)
  target_compile_definitions(slam_server PRIVATE SLAM_SERVER_ZXING_SYSTEM_HEADERS)
endif()
if(LIBHEIF_FOUND)
  target_compile_definitions(slam_server PRIVATE SLAM_SERVER_HAVE_LIBHEIF)
  target_include_directories(slam_server PRIVATE ${LIBHEIF_INCLUDE_DIRS})
  target_compile_options(slam_server PRIVATE ${LIBHEIF_CFLAGS_OTHER})
  target_link_libraries(slam_server ${LIBHEIF_LDFLAGS})
  message(STATUS "slam_server HEIC fallback enabled via libheif ${LIBHEIF_VERSION}")
else()
  message(STATUS "slam_server HEIC fallback disabled; install libheif-dev for HEIC uploads when OpenCV lacks HEIF support")
endif()

if(AUKI_LIGHTGLUE_AVAILABLE)
  target_compile_definitions(slam_server PRIVATE AUKI_LIGHTGLUE_AVAILABLE)
  target_include_directories(slam_server PRIVATE "${LIGHT_GLUE_CPP_DIR}/include")
  add_executable(smoke_aliked_lightglue Examples/Monocular/smoke_aliked_lightglue.cc)
  target_compile_definitions(smoke_aliked_lightglue PRIVATE AUKI_LIGHTGLUE_AVAILABLE)
  target_include_directories(smoke_aliked_lightglue PRIVATE "${LIGHT_GLUE_CPP_DIR}/include")
  target_link_libraries(smoke_aliked_lightglue PRIVATE
    "-Wl,--no-as-needed"
    ${PROJECT_NAME}
    ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so
    "-Wl,--as-needed")
  target_compile_options(smoke_aliked_lightglue PRIVATE ${TORCH_CXX_FLAGS} "-std=c++20")
  set_target_properties(smoke_aliked_lightglue PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON)

  add_executable(bench_aliked_topk Examples/Monocular/bench_aliked_topk.cc)
  target_compile_definitions(bench_aliked_topk PRIVATE AUKI_LIGHTGLUE_AVAILABLE)
  target_include_directories(bench_aliked_topk PRIVATE "${LIGHT_GLUE_CPP_DIR}/include")
  target_link_libraries(bench_aliked_topk PRIVATE
    "-Wl,--no-as-needed"
    ${PROJECT_NAME}
    ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so
    "-Wl,--as-needed")
  target_compile_options(bench_aliked_topk PRIVATE ${TORCH_CXX_FLAGS} "-std=c++20")
  set_target_properties(bench_aliked_topk PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/Examples/Monocular")

  add_executable(bench_batch_aliked_lg Examples/Monocular/bench_batch_aliked_lg.cc)
  target_compile_definitions(bench_batch_aliked_lg PRIVATE AUKI_LIGHTGLUE_AVAILABLE)
  target_include_directories(bench_batch_aliked_lg PRIVATE "${LIGHT_GLUE_CPP_DIR}/include")
  target_link_libraries(bench_batch_aliked_lg PRIVATE
    "-Wl,--no-as-needed"
    ${PROJECT_NAME}
    ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so
    "-Wl,--as-needed")
  target_compile_options(bench_batch_aliked_lg PRIVATE ${TORCH_CXX_FLAGS} "-std=c++20")
  set_target_properties(bench_batch_aliked_lg PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/Examples/Monocular")
endif()

#Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

add_executable(mono_inertial_euroc
        Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi
        Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

# CUSTOM
add_executable(mono_inertial_dmt
        Examples/Monocular-Inertial/mono_inertial_dmt.cc)
target_link_libraries(mono_inertial_dmt ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

add_executable(stereo_inertial_euroc
        Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})