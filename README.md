# VIF-Stereo
**Authors:** [Weihan Wang](https://github.com/wwtx9/),[Junhong Chen](https://github.com/JohnChen-PCL)

VIF-Stereo is a Visual-Inertial(Stereo Cameras and Inertial Measurement Unit(IMU)) SLAM system based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) and [VINS](https://github.com/HKUST-Aerial-Robotics/VINS-Mono). VIF-Stereo system includes three main parts: Visual-Inertial alignment, Visual-Inertial system initialization and tightly-coupled Visual-Inertial system optimization. Current Version of VIF-Stereo has finished Visual-Inertial alignment and Visual-Inertial system initialization, but tightly-coupled Visual-Inertial system optimization section is still working.


# 1. Prerequisites
We have tested the library in **Ubuntu 12.04**, **14.04** and **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

# 2. Building VIF-Stereo System

Clone the repository:
```
git clone https://github.com/wwtx9/VIF-Stereo.git
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd VIF-Stereo
chmod +x build.sh
./build.sh
```

# 3. Stereo Examples

## EuRoC Dataset
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/mav0/cam0/data PATH_TO_SEQUENCE/mav0/cam1/data PATH_TO_SEQUENCE/mav0/cam0/data.csv PATH_TO_SEQUENCE/mav0/imu0/data.csv
```



<!-- href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a-->




