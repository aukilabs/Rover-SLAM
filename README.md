# A real-time, robust and versatile visual-SLAM framework based on deep learning networks

# Prerequisites
We have tested the library in **Ubuntu 20.04**, with the following hardware and software configurations:

- **CPU**: Intel Core i7-10700K
- **GPU**: NVIDIA GeForce RTX 3080
- **CUDA Version**: 11.8



## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Download and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
**Required at least 3.0. Tested with OpenCV 3.4.1**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## ONNXRuntime
**Required onnxruntime-linux-x64-gpu-1.16.3** and Modify line 63 of the CMakeLists.txt to the current location of ONNXRuntime library.


## ROS (optional)

We provide some examples to process input of a monocular, monocular-inertial, stereo, stereo-inertial or RGB-D camera using ROS. Building these examples is optional. These have been tested with ROS Melodic under Ubuntu 18.04.



## Download Examples Folder
Download ["Examples" ](https://pan.baidu.com/s/1dd6k_Gf8mEjbiyli31_Yeg?pwd=p5y8) and unzip in ROVER-SLAM/ .

## Download Dbow File
Download ["voc_binary_tartan_8u_6.zip"](https://pan.baidu.com/s/1dd6k_Gf8mEjbiyli31_Yeg?pwd=p5y8), and unzip in ROVER-SLAM/Vocabulary/ .

## ALIKED + LightGlue (MrNeRF stack, default when LibTorch is enabled)

Tracking uses the vendored MrNeRF LibTorch stack (`third_party/Light_Glue_CPP`: ALIKED `aliked-n16`, top_k≈1024, + LightGlue matcher) when CMake finds LibTorch. SuperPoint/LightGlue ONNX are not used on that path.

**Build requirements (GPU host):**

- **LibTorch:** set `LIBTORCH_ROOT` to a CUDA LibTorch install (CMake env var or `-DLIBTORCH_ROOT=...`). If unset, configure skips `auki_lightglue_lib` and falls back to the legacy ONNX SuperPoint/LightGlue path.
- **CUDA:** **12.1 or newer** (required by `cmake/modules/light_glue_cpp.cmake`).
- **Model weights:** place MrNeRF checkpoints under `third_party/Light_Glue_CPP/models/` — at minimum `aliked-n16.pt` and `aliked_lightglue.pt` (shipped with the vendor tree).

Example configure:

```bash
export LIBTORCH_ROOT=/opt/libtorch   # directory containing share/cmake/Torch/TorchConfig.cmake
mkdir build && cd build
cmake .. -DLIBTORCH_ROOT=$LIBTORCH_ROOT
make -j$(nproc)
```

**Smoke test (GPU host):**

After a LibTorch-enabled build, run the ALIKED+LightGlue harness from `Examples/Monocular` (default images: `frame_100_original.png` and `frame_101_original.png` in the repo root):

```bash
cd build
make smoke_aliked_lightglue
cd ../Examples/Monocular
./smoke_aliked_lightglue
# optional: ./smoke_aliked_lightglue /path/to/a.png /path/to/b.png [cuda|cpu]
```

Expect non-zero keypoint counts on both images and a positive match count. The target is omitted when `LIBTORCH_ROOT` is unset at configure time.

**Runtime (v1):**

- Set **`loopClosing: 0`** in your settings YAML. Loop closure and relocalization are **deferred** until a follow-on EigenPlaces retrieval initiative; tracking-only operation is supported.
- The **vocabulary file is still required** on the command line (`voc_binary_tartan_8u_6.yml.gz` or equivalent SuperPoint DBoW vocab). Startup loads this file even with loop closing off; removing the CLI argument is out of scope for v1.

See `Examples/Monocular/README_slam_server.md` for `slam_server`-specific settings and examples.

# Building Rover-SLAM library and examples

Clone the repository:
```
git clone https://github.com/zzzzxxxx111/Rover-SLAM.git
```


```
cd Rover-slam
mkdir build
cd build
cmake ..
make -j12
```


# Running 

## Euroc-Monocular:
```
./Examples/Monocular/mono_euroc  Vocabulary/voc_binary_tartan_8u_6.yml.gz Examples/Monocular/EuRoC.yaml /media/xiao/data3/slamdataset/euroc/V202 /media/xiao/data3/learning-slam/Rover-slam/Examples/Monocular/EuRoC_TimeStamps/V202.txt
```

## Euroc-Monocular-Inerial:

```
./Examples/Monocular-Inertial/mono_inertial_euroc  Vocabulary/voc_binary_tartan_8u_6.yml.gz Examples/Monocular-Inertial/EuRoC.yaml /media/xiao/data3/slamdataset/euroc/V203 media/xiao/data3/learning-slam/Rover-slam/Examples/Monocular-Inertial/EuRoC_TimeStamps/V203.txt
```



## TUM-Monocular-Inertial

```
./Examples/Monocular-Inertial/mono_inertial_tum_vi Vocabulary/voc_binary_tartan_8u_6.yml.gz Examples/Monocular-Inertial/TUM_512.yaml /media/xiao/data3/slamdataset/dataset-corridor3_512_16/mav0/cam0/data Examples/Monocular-Inertial/TUM_TimeStamps/dataset-corridor3_512.txt Examples/Monocular-Inertial/TUM_IMU/dataset-corridor3_512.txt dataset-corridor3_512_monoi
```
## Euroc-Stereo-Inertial

```
 ./Examples/Stereo-Inertial/stereo_inertial_euroc /media/xiao/data3/learning-slam/ORB_SLAM3_detailed_comments/Vocabulary/voc_binary_tartan_8u_6.yml.gz Examples/Stereo-Inertial/EuRoC.yaml /media/xiao/data3/slamdataset/euroc/V203 /media/xiao/data3/learning-slam/ORB_SLAM3_detailed_comments/Examples/Stereo/EuRoC_TimeStamps/V203.txt V203_si
```      
The rest of the operations are the same as ORB-SLAM3



# Acknowledgments

The completion of this project would not have been possible without the support and contributions of the following open-source projects and tools. We extend our sincere gratitude to:

1. **ORB-SLAM3**  
   

2. **AIRVO**  
  

3. **SP-Loop**  

4. **ORB_SLAM3_detailed_comments**  

5. **SuperPoint_SLAM**
