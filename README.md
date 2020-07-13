# UVDAR drivers and processing [![Build Status](https://travis-ci.com/ctu-mrs/uvdar.svg?branch=master)](https://travis-ci.com/ctu-mrs/uvdar)

![](.fig/thumbnail.jpg)

## Description

* the UVDAR system is a visual mutual relative localization system for cooperating micro-scale UAVs
* based on ultraviolet-sensitive cameras and blinking ultraviolet markers
* can be used both indoors and outdoors without infrastructure
* robust to a variety of lighting conditions

## System requirements

#### Hardware:
* One or more calibrated (Using the OCamCalib model) grayscale camera sensors with ultraviolet bandpass filters. In our setup these are:
  * mvBlueFOX MLC200wG cameras
  * Sunnex DSL215 lenses with ~180 degrees of horizontal FOV 
  * Midopt BP365-R6 filters with our custom holder between the sensor and the lens

* Blinking ultraviolet LEDs (395nm) attached to extreme points of the target UAVs. In our setup, these are:
  * ProLight Opto PM2B-1LLE
  * Attached to the ends of the arms of the UAVs, respectively on the top of the UAVs for "beacons"
  * For quadrotors, the markers comprise two LEDs each, rotated 90&deg; from each other in the "yaw" axis of the UAV

#### Software
  * [ROS (Robot Operating System)](https://www.ros.org/) Melodic Morenia
  * [mrs_lib](https://github.com/ctu-mrs/mrs_lib) - ROS package with utility libraries used by the MRS group
  * [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs) - ROS package with message types used by the MRS group
  * [bluefox2](https://github.com/ctu-mrs/bluefox2) - ROS package providing interface with mvBlueFOX cameras


#### For testing in simulation
  * [mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system) Our ROS-based ecosystem for flying and testing multi-UAV systems

## Installation
Install the dependencies.
Clone this repository into a ROS workspace as a package.
If you are using the `mrs_modules` meta package (currently only accessable internally to MRS staff, to be released at later date), this repository is already included.
Build the package using catkin tools (e.g. `catkin build uvdar_core`)

## Testing
In order to test the system in simulation, install all the dependencies including `mrs_uav_system` (Above) and run one of the two start scripts in the [scripts](scripts/) folder:
  * For testing separation of units based on position and beacons use [beacon_test.sh](scripts/beacon_test.sh)
  * For testing separation of units based on different blinking frequencies [multi_frequency_test.sh](scripts/multi_frequency_test.sh)

## Node description
The package comprises multiple ROS nodes (N) and nodelets (n):
  * [UVDARDetector](src/detector.cpp) - n - detects bright points from the UV camera image. These are used as candidates for UV LED markers
  * [UVDARBlinkProcessor](src/blink_processor.cpp) - n - Extracts blinking frequencies and image positions of the markers detected previously
  * [UVDARPoseCalculator](src/uav_pose_calculator.cpp) - N - Calculates approximate pose and error covariances of the MAVs carrying the UV LED markers. Note, that this is an example for the specific layouts on quadrotors and hexarotors we use. If you need a different layout, you will also need to write your custom pose calculation
  * [UVDARKalman](src/filter.cpp) - N - Filters out sets of detected poses with covariances based on positions or the included identities. This filtering occurs in a selected coordinate frame

  * [UVDARBluefoxEmulator](src/bluefox_emulator.cpp)  - n - Generates an image stream similar to the output of our Bluefox cameras with UV bandpass filters (above). This image is currently rudimentary, with background of a constant shade of grey and white circles where the markers appeared. The function of this node depends on our Gazebo plugin (TODO), with which it needs to communicate
  * [MaskGenerator](src/mask_generator.cpp) - N - Generates masks for specific camearas on specific MAVs. This is necessary to suppress detections of marers on the given observer, as well as reflection from e.g. its metallic surfaces in front of the camera. The masks can be also generated manually.
  * [FrequencySetter](src/frequency_setter.cpp) - N - Sends commands to our controller boards that set the frequencies of the blinking UV LEDs on the current MAV

## Acknowledgements

### MRS group
This work would not be possible without the hard work, resources and support from the [Multi-Robot Systems (MRS)](http://mrs.felk.cvut.cz/) group from Czech Technical University.

### Included libraries
This package contains the following third-party libraries by the respective authors:
  * [OCamCalib](https://sites.google.com/site/scarabotix/ocamcalib-toolbox) calibration system by Davide Scaramuzza - only the C++ sources
  * [P3P](https://www.laurentkneip.com/software) library by Laurent Kneip
