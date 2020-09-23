# Multispectral Processing - Multi-modal Data Processing and Implementation for Vineyard Analysis

Multispectral Processing is an implementation in ROS Melodic for Multi-modal Data Processing and Implementation for Vineyard Analysis. The main focus of the project is the development
of a method for the registration of multi-modal images in order to obtain a three-dimensional
reconstruction of the vine enriched with photometric or radiometric data. Furthermore, an
artificial intelligence module is developed to jointly process images from the different modalities
for a detailed analysis of the plant's condition.

## Table of Contents

[Requirements](#requirements)

[Pipeline](#pipeline)

[Packages Installation](#packages-installation)

[Source Files](#source-files)

[Launch Files](#launch-files)

[Resources](#resources)

[Execution](#execution)

[Demo Experiments](#demo-experiments)

[Figures](#figures)

[License](#license)

## Requirements

### Software

* ROS Melodic Morenia.
* Ubuntu 18.04.5 LTS.

### Hardware

* CMS-V GigE Silios Multispectral Camera.
* Microsoft Kinect V2 Sensor: RGB-D Camera.

## Pipeline

<p align="center">
    <img src="/data/images/img15.png" width="60%" title="Pipeline" />
</p>

## Packages Installation

Be sure that you have installed the melodic version of the packages below.

* [ueye_cam](http://wiki.ros.org/ueye_cam): ROS package that that wraps the driver API for UEye cameras by IDS Imaging Development Systems GMBH.

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/anqixu/ueye_cam.git
    $ cd ~/catkin_ws
    $ catkin_make
    ```

* [iai_kinect2](https://github.com/code-iai/iai_kinect2): Package that provides tools for Kinect v2 such as bridge between kinect and ROS, cameras calibration, etc.

* [libfreenect2](https://github.com/OpenKinect/libfreenect2): Drivers for Kinect v2.

* [image_pipeline](http://wiki.ros.org/image_pipeline): Package that provides functionalities about cloud point, calibration, etc.

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/ros-perception/image_pipeline.git
    $ cd ~/catkin_ws
    $ catkin_make
    ```

* [rosbridge_suite](http://wiki.ros.org/rosbridge_suite): ROS package that provides a JSON API to ROS functionality for non-ROS programs.

    `$ sudo apt-get install ros-melodic-rosbridge-server`

* [rviz](http://wiki.ros.org/rviz): 3D visualization tool for ROS.

* [rtabmap_ros](http://wiki.ros.org/rtabmap_ros): A RGB-D SLAM approach with real-time constraints.

    `$ sudo apt-get install ros-melodic-rtabmap-ros`

## Source Files

* band_separator.cpp: C++ node for multispectral image separation, pre-processing and processing. Provides GUI for multiple tasks.
* band_separator.py: Python node for multispectral image separation, pre-processing and processing. Provides GUI for multiple tasks.
* backup.cpp: C++ node for saving single frames or stream of frames. 
* backup.py: Python node for saving single frames or stream of frames.
* experiments.cpp: This node publishes images when to the topic that band_separator node subscribes. It can be used when no camera is available.
* features_registraor.cpp: C++ node that detects features from 2 images and align them.
* features_registraor.py: Python node that detects features from 2 images and align them.
* corners_registraor.cpp: C++ node that detects features from 2 images and align them.
* corners_registraor.py: Python node that detects features from 2 images and align them.
* synchronizer.cpp: C++ node that subscribes to image topics and publishes them after synchronization.
* synchronizer.py: Python node that subscribes to image topics and publishes them after synchronization.
* calibrator.py: Node that performs calibration.
* stereo_calibrator.py: Node that performs stereo calibration.
* tf_nde.cpp: Tranformation node with C++.
* tf_nde.py: Tranformation node with Python.

## Launch Files

* cms_cpp.launch: Run multispectral camera, pre-processing functionalities with C++, connection between camera and controller.
* cms_py.launch: Run multispectral camera, pre-processing functionalities with Python, connection between camera and controller.
* camera_configurator: Run multispectral camera, connection between camera and controller.
* kinect2_bridge.launch: Run kinect.
* point_cloud_generator: Generate point clouds from given topics.
* ueye_camera_gige.launch: Run multispectral nodelet to turn on the camera.
* stereo_calibration.launch: Calibtation node for stereo cameras.
* calibration.launch: Run the calibration node for the multispectral camera.
* registration_approach1_cpp.launch: Image registration launch file for C++ node with approach 1.
* registration_approach1_py.launch: Image registration launch file for Python node with approach 1.
* registration_approach2_cpp.launch: Image registration launch file for C++ node with approach 2.
* registration_approach2_py.launch: Image registration launch file for Python node with approach 2.

## Resources

* fps_log.yaml: Log file for FPS.
* parameters.yaml: Manufacturer parameters such as serial number, cross-talk correction coefficients, etc.
* multispectral_camera.yaml: Calibration parameters for multispectral camera.
* left.yaml: Calibration parameters for stereo vision.
* right.yaml: Calibration parameters for stereo vision.
* homography1.yaml: This file contains the perspective transformation matrix between the images for approach 1.
* homography2.yaml: This file contains the perspective transformation matrix between the images  for approach 2.
* data folder: This folder contains multiple images for experiments.

## Execution

### Functionalities

1. Multispectral Camera:

    * Acquisition & Band Separation.

    * Flat-field Correction.

    * White Balance Normalization.

    * Crosstalk Correction.

    * Vegetation Indeces Calculation NDVI, MCARI, MSR, SAVI, TVI, etc.

2. Both Cameras:

    * Cameras Geometric Calibration.

    * Multi-modal Image Registration.

    * 3D Reconstruction.

### Permissions

Change permissions to all python files to be executable with the command below:

```
$ roscd multispectral_processing/src
$ chmod +x *.py
```

### Preparation for Image Acquisition

Follow the steps below to succeed the best image acquisition.

1. Connection of multispectral camera, kinect V2 sensor and PC with ROS installation.
2. Sensor alignment.
3. Adjusting the optics:
    * Adjust the "Focus or Zoom" of the lens on an object at the same distance as the vine.
    * Adjust the "Aperture" of the lens.
4. Set acquisitions parameters
    * Gain (not auto gain, lower is better).
    * Exposure time (we can change it as convenience).
    * Framerate.
    * Others.
5. Pre-processing parameters:
	* Set white balance, white reference.
	* Set crosstalk correction or not. 
	* Set flatfield correction or not.
6. Start one of the registration approaches as described below to register Homographies (rotations, translations, scale). Be sure theat the sensors are fixed. "DO NOT TOUCH SENSORS".
7. Save a single frame or multiple frames when running image registration in no capture mode, by using the command below:

    `$ rosrun multispectral_processing backup.py`

    or
    
    `$ rosrun multispectral_processing backup`

### Image Registration

For the whole implementation is used C++ and Python code. Every node is developed with C++ and with Python respectively. Image registration is performed between multispectral camera and Kinect V2 camera.

1. Multispectral camera and kinect cameras image registration, via feature detection (C++). Edit args="capture" to start corners capturing or args="nocapture" to start publishing.

    `<node name="features_registrator" pkg="multispectral_processing" type="features_registrator" args="nocapture" output="screen"/>`

    and run

    `$ roslaunch multispectral_processing registration_approach1_cpp.launch`

2. Multispectral camera and kinect cameras image registration, via feature detection (Python). Edit args="capture" to start corners capturing or args="nocapture" to start publishing.

    `<node name="features_registrator" pkg="multispectral_processing" type="features_registrator.py" args="nocapture" output="screen"/>`

    and run

    `$ roslaunch multispectral_processing registration_approach1_py.launch`

3. Multispectral camera and kinect cameras image registration, via chessboard coreners detection (C++). Edit args="capture" to start corners capturing or args="nocapture" to start publishing.

    `<node name="corners_registrator" pkg="multispectral_processing" type="corners_registrator" args="nocapture" output="screen"/>`

    and run

    `$ roslaunch multispectral_processing registration_approach2_cpp.launch`

4. Multispectral camera and kinect cameras image registration, via chessboard coreners detection  (Python). Edit args="capture" to start corners capturing or args="nocapture" to start publishing.

    `<node name="corners_registrator" pkg="multispectral_processing" type="corners_registrator.py" args="nocapture" output="screen"/>`

    and run

    `$ roslaunch multispectral_processing registration_approach2_py.launch`

### 3D Reconstruction

For mapping by using rtabmap_ros package:

1. Run one of the registration approaches with args="nocapture".
2. Run the command to start rtabmap_ros package:

    `$ roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" rgb_topic:=/multispectral/image_mono depth_topic:=/multispectral/image_depth camera_info_topic:=/multispectral/camera_info approx_sync:=false`

    or for external odometry use:

    `$ roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" rgb_topic:=/multispectral/image_mono depth_topic:=/multispectral/image_depth camera_info_topic:=/multispectral/camera_info approx_sync:=false visual_odometry:=false odom_topic:=/my_odometry`

    and replace odom_topic:=/my_odometry with the external odometry topic.

## Demo Experiments

These experiments include only the imagees of the multispectral camera and the included processes. Run experiments with the already captured images located in [/data/simulation](/data/simulation) folder and follow the steps below:

1. Comment the includes below in [cms_cpp.launch](/launch/cms_cpp.launch) or [cms_py.launch](/launch/cms_py.launch) file.

    ```
    <!-- <include file="$(find multispectral_processing)/launch/kinect2_bridge.launch"/> -->
    <!-- <include file="$(find multispectral_processing)/launch/ueye_camera_gige.launch"/> -->
    ```
2. Uncomment the include of [experiments.cpp](/src/experiments.cpp) node.

    `<node name="experiments" pkg="multispectral_processing" type="experiments" args="7" output="screen"/>`

3. Choose the dataset that you want by changing the "args" value.
4. Run [cms_cpp.launch](/launch/cms_cpp.launch) or [cms_py.launch](/launch/cms_py.launch) file.

## Figures

### Sensors Position

<p align="center">
    <img src="/data/images/img1.png" width="30%" title="Sensors Position" />
</p>

### Captured Image & Bands by the Multispectral Camera and OpenCV UI

<p align="center">
    <img src="/data/images/img2.png" width="45%" title="Captured Image" />
    <img src="/data/images/img3.png" width="45%" title="Captured Bands" />
</p>

### Captured RGB Image by the Kinect V2 Sensor, Captured Bands by the Multispectral Camera

<p align="center">
    <img src="/data/images/img4.png" width="45%" title="Captured RGB Image" />
    <img src="/data/images/img5.png" width="45%" title="Captured Bands" />
</p>

### NDVI calculation, Colored vegetation, Colored Vegetation After Crosstalk Correction

<p align="center">
    <img src="/data/images/img6.png" width="32%" title="NDVI" />
    <img src="/data/images/img7.png" width="32%" title="Colored Vegetation" />
    <img src="/data/images/img8.png" width="32%" title="Colored Vegetation After Crosstalk Correction" />
</p>

### Background Subtraction by using Otsu's method

<p align="center">
    <img src="/data/images/img9.png" width="45%" title="Segmentation" />
    <img src="/data/images/img10.png" width="45%" title="Erosion and Dilation" />
</p>

### Image Registration with Feature Matching

<p align="center">
    <img src="/data/images/img11.png" width="60%" title="Image Registration with Feature Matching" />
</p>

### Image Registration with Corner Matching

<p align="center">
    <img src="/data/images/img12.png" width="60%" title="Image Registration with Corner Matching" />
</p>

### 3D Reconstruction

<p align="center">
    <img src="/data/images/img13.png" width="60%" title="3D Reconstruction 1" />
</p>

<p align="center">
    <img src="/data/images/img14.png" width="60%" title="3D Reconstruction 2" />
</p>

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.