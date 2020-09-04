# Multispectral_processing - Multi-modal Data Processing and Implementation for Vineyard Analysis

Multispectral Processing is an implementation in ROS Melodic for Multi-modal Data Processing and Implementation for Vineyard Analysis. The main focus of the project is the development
of a method for the registration of multi-modal images in order to obtain a three-dimensional
reconstruction of the vine enriched with photometric or radiometric data. Furthermore, an
artificial intelligence module is developed to jointly process images from the different modalities
for a detailed analysis of the plant's condition.

# Packages to be installed

* ueye_cam: ROS nodelet for multispectral camera that wraps drivers and API.
* iai_kinect2: Package that provides tools for Kinect v2 such as bridge, calibration, etc.
* libfreenect2: Drivers for Kinect v2.
* image_pipeline: Package that provides functionalities about cloud point, calibration, etc.
* rosbridge_suite: Connection between ROS and camera controller.

# List of source files

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

# List of launch files

* cms_cpp.launch: Run multispectral camera, pre-processing functionalities with C++, connection between camera and controller.
* cms_py.launch: Run multispectral camera, pre-processing functionalities with Python, connection between camera and controller.
* camera_configurator: Run multispectral camera, connection between camera and controller.
* kinect2_bridge.launch: Run kinect.
* point_cloud_generator: Generate point clouds from given topics.
* ueye_camera_gige.launch: Run multispectral nodelet to turn on the camera.
* stereo_calibration.launch: Calibtation node for stereo cameras.
* calibration.launch: Run the calibration node for the multispectral camera.
* registtration_approach1_cpp.launch: Image registration launch file for C++ node with approach 1.
* registtration_approach1_py.launch: Image registration launch file for Python node with approach 1.
* registtration_approach2_cpp.launch: Image registration launch file for C++ node with approach 2.
* registtration_approach2_py.launch: Image registration launch file for Python node with approach 2.

# Resources

* fps_log.yaml: Log file for FPS.
* parameters.yaml: Manufacturer parameters such as serial number, cross-talk correction coefficients, etc.
* multispectral_camera.yaml: Calibration parameters for multispectral camera.
* left.yaml: Calibration parameters for stereo vision.
* right.yaml: Calibration parameters for stereo vision.
* homography1.yaml: This file contains the perspective transformation matrix between the images for approach 1.
* homography2.yaml: This file contains the perspective transformation matrix between the images  for approach 2.
* data folder: This folder contains multiple images for experiments. 