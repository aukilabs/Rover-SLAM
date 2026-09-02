# LibTorch discovery for MrNeRF / Light_Glue_CPP (Rover-SLAM ALIKED+LightGlue).
# When unset, auki_lightglue_lib is not built; set LIBTORCH_ROOT on the GPU build host.

if(DEFINED LIBTORCH_ROOT)
    set(LIBTORCH_ROOT "${LIBTORCH_ROOT}" CACHE PATH "LibTorch install prefix")
elseif(DEFINED ENV{LIBTORCH_ROOT})
    set(LIBTORCH_ROOT "$ENV{LIBTORCH_ROOT}" CACHE PATH "LibTorch install prefix")
elseif(EXISTS "/opt/libtorch/share/cmake/Torch/TorchConfig.cmake")
    set(LIBTORCH_ROOT "/opt/libtorch" CACHE PATH "LibTorch install prefix")
endif()

if(LIBTORCH_ROOT)
    list(PREPEND CMAKE_PREFIX_PATH "${LIBTORCH_ROOT}")

    set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES
        CMAKE_C_COMPILER
        CMAKE_CXX_COMPILER
        CMAKE_CUDA_COMPILER
        CMAKE_CUDA_HOST_COMPILER
        CUDA_TOOLKIT_ROOT_DIR
        CUDAToolkit_ROOT)

    if(NOT TORCH_CUDA_ARCH_LIST)
        set(TORCH_CUDA_ARCH_LIST "12.0" CACHE STRING "LibTorch CUDA arch list (skip nvcc probe)")
    endif()

    if(NOT CMAKE_CUDA_COMPILER)
        if(DEFINED ENV{CUDA_HOME})
            set(CMAKE_CUDA_COMPILER "$ENV{CUDA_HOME}/bin/nvcc" CACHE FILEPATH "CUDA compiler")
        elseif(EXISTS "/usr/local/cuda-12.9/bin/nvcc")
            set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.9/bin/nvcc" CACHE FILEPATH "CUDA compiler")
        elseif(EXISTS "/usr/local/cuda/bin/nvcc")
            set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "CUDA compiler")
        endif()
    endif()
    if(NOT CMAKE_CUDA_HOST_COMPILER AND CMAKE_CXX_COMPILER)
        set(CMAKE_CUDA_HOST_COMPILER "${CMAKE_CXX_COMPILER}" CACHE FILEPATH "CUDA host compiler")
    endif()

    if(NOT TARGET CUDA::nvToolsExt)
        find_library(AUKI_nvToolsExt_LIBRARY nvToolsExt
            PATHS
            "${CUDAToolkit_LIBRARY_DIR}"
            /usr/lib/x86_64-linux-gnu
            /usr/local/cuda/lib64
            /usr/local/cuda-12.9/lib64)
        if(AUKI_nvToolsExt_LIBRARY)
            add_library(CUDA::nvToolsExt UNKNOWN IMPORTED)
            set_target_properties(CUDA::nvToolsExt PROPERTIES
                IMPORTED_LOCATION "${AUKI_nvToolsExt_LIBRARY}")
        endif()
    endif()
    find_package(Torch REQUIRED)
    message(STATUS "LibTorch: ${TORCH_INSTALL_PREFIX} (Torch ${Torch_VERSION})")
    set(AUKI_LIBTORCH_FOUND TRUE)
else()
    message(
        STATUS
        "LIBTORCH_ROOT unset; skipping MrNeRF LightGlue (set -DLIBTORCH_ROOT= or env LIBTORCH_ROOT to enable auki_lightglue_lib)")
endif()

set(LIGHT_GLUE_CPP_DIR
    "${PROJECT_SOURCE_DIR}/third_party/Light_Glue_CPP"
    CACHE PATH "MrNeRF Light_Glue_CPP vendor root")

if(EXISTS "${LIGHT_GLUE_CPP_DIR}/CMakeLists.txt")
    message(STATUS "Light_Glue_CPP tree: ${LIGHT_GLUE_CPP_DIR}")
else()
    message(
        WARNING
        "Light_Glue_CPP not found at ${LIGHT_GLUE_CPP_DIR}")
endif()
