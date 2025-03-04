## Relevant people
For issues with UVDAR elements - cameras, filters, lenses, and `uvdar_core` software, as well as software integration of the UVDAR-UWB system, contact me - `viktor.walter@fel.cvut.cz`.

For issues with GPU acceleration - contact Vojtěch Vrba - `vojtech.vrba@cvut.cz`.

For issues with UWB, including drivers in the `spi_uwb_controller` package, contact Vít Petřík - `petrivi2@student.cvut.cz`.

For general hardware issues, contact Dan Heřt - `daniel.hert@cvut.cz`.

For general software issues, including Portainer and Docker stuff, contact Tomáš Báča - `tomas.baca@cvut.cz`.

# System specifics

## Initial setup
After the drone is otherwise set up with everything up to the Portainer, follow the "Installation for RoboFly" steps in the main `README.md` to add the necessary elements for the UVDAR-UWB system.


## Configuration
Configs specific to individual RoboFly units are found in `uav_custom_files/data/robofly/#####/`, with example files in `#####`=`f1f8c2`.

## IDs
UVDAR and UWB each have their own numerical ID.
For UVDAR, this ID corresponds to the blinking sequence from the set of 27 (currently - see `uvdar_core/config/selected.txt`) blinking identities.
For UWB, this is the PAN MAC address, of which there are thousands available.
Both sets are starting from 0, and I've been setting it up such that each unit has the two IDs equal to each other.
For each unit, configure these in `uav_custom_files/data/robofly/#####/uvdar/set_ids.sh` which defines what the current unit's blinking sequence and UWB MAC will be set to.
Then for estimating other unit's locations, you need to define these ID pairs in `uvdar_core/config/uwb_uvdar_fuser/target_ids.yaml`.
Note that the `uvdar_ids` is an array - this is legacy from testing with UVDAR drones with multiple different blinking ID.

## Camera calibrations
The system uses OCamCalib calibration files for the cameras.
Ideally, these should be generated for each Arducam-Filter-Lens system using our fork of the OCamCalib toolkit, but for testing purposes I've been copying the calibration files for the reference units.
These should be in the `uav_custom_files` repo, and named as `uav_custom_files/data/robofly/#####/camera_calibrations/calib_results_rp_uv_front.txt` and `uav_custom_files/data/robofly/#####/camera_calibrations/calib_results_rp_uv_back.txt` for the two cameras.

## 3D position filtering
The system fuses bearing information from the UVDAR camera outputs and distance information from the UWB ranging modules using our variant of the Linear Kalman Filter that we call the "Degenerate Kalman Filter", allowing for input covariances with infinite eigenvalues, such as for filtering 3D position only using the bearing along an infinite line.

The bearings are obtained by combining the image pixel positions of the annotated markers with the camera calibrations.

The distance estimates are used directly from the ranging messages, but in the filter they are represented by a flat (disk shaped) 3D  covariances perpendicular to the bearing of the latest estimate.

The two types of measurement are entering the filter asynchronously.

# Quirks and issues to be addressed


## Power bleed-over and Battery dependence
When the Raspi PC is powered with the USB-C adapter, it seems to activate the rest of the RoboFly unit to some extent.
This means that the motors "jitter" a bit, the ESCs beep, etc.
On the other hand, the UVDAR-UWB module does not work without battery being attached, on the account of being powered by 12V input that is not generated otherwise.

I suggest to
- more strictly separate the actuation elements of the system from the PC USB-C power input
- add a boost converter to generate the 12V even without battery to allow for testing on the table without batteries

## UWB module brownout
After the battery is plugged in, the UWB module tends not to work properly until the PC is restarted through software.
This issue seems to be (but I have no statistics) correlated with cases where pluggin in the battery activated the PC into the on (green light) state.

Possible solutions
- de-bounce and filter the 12V power line to the UVDAR-UWB module
- somehow disconnect the wire bus until the rest of the system is started up
- find a software-based reset method

## UWB software not starting the first time
May be related to the issue above.
Normally, the `UWB_ID` is passed as a parameter to the driver reload script `~/git/spi_uwb_controller/driver/reload.sh` though the launch file `uvdar_core/launch/uwb_uvdar_fuser.launch`
Non-functional UWB communication can be detected by the topic `uav##/radar_nodelet/range` not outputting anything or by the relative pose measuremens reporting nonsensical distances, but correct relative bearings.
If the UWB modules do not initialize it may be necessary to call the driver reload script manually first, with the relevant UWB ID (7 in the example below) as a parameter:
```
cd ~/git/spi_uwb_controller/driver/
./reload.sh 7
```
Then check if the relevant lines of the output of `dmesg -Hw` contain `0xdeca` and not all `0x0000` values.
Sometimes, this may work after restart.

Suggestions
- Figure out the causes
- add delays before calling the reload script in the launcher file

## GPU acceleration and kernel modules
UVDAR requires acceleration by at least an integrated GPU. If this is not available, the system will usually still work by defaulting to CPU processing, but the load on the CPU will be significant, and so such cases should be avoided.
Due to the Raspberry Pi 5 being relatively new, in conjunction with our using of dockerized Ubuntu of an older version, the access to the GPU acceleration didn't work by default.
This is the reason for installing the third-party mesa drivers (kisak) in the instructions in `README.md`.
If the system changes, this may stop working and so fixes will be needed.
In other UAV units, I was addressing this also by installing (and even compiling) a newer Linux kernel, which will of course also necessitate re-building of the custom kernel modules - see below

## UWB controller kernel modules
In order to properly communicate with the UWB module, you need to set up a set of kernel modules to load on the start of the system, including two that you need to build yourself.
These are located in the `spi_uwb_controller` repo.
To use them, first build and install the `IEEE802154` module following the "Easier approach without needing kernel source" section in `spi_uwb_controller/kernel/README.md`.
Next, build the `dw3000-drv` kernel module and insert the relevant kernel modules into `/etc/modules-load.d/modules.conf`, according to the manual in `spi_uwb_controller/driver/README.md`
You also need `wpan-tools` intalled in the OS.
After these steps are done, calling the `spi_uwb_contooler/driver/reload.sh` script should work.

## UWB "unipollarity"
Currently, the UWB ranging works by one unit ("initiator") sending out a messge, and another unit ("responder") sending a message back. 
This should be followed by one additional such exchange (as far as I understand), providing both the units with an estimate of the distance to the other based on time of flight.
Now, this means that there is an implicit hierarchy. If both units are set to send out initiation messagaes, one of them gets jammed by the other.
Our current solution is such that the UAV with the higher UWB ID (= PAN MAC address) will be the initiator, and the lower one will be the responder.
In other words, the unit with higher MAC address ignores boradcast messges from units with lower MAC addresses.
If this is seen as insufficient, talk with Vít Petřík.

## Portainer loading times
The portainer system take long time - even a few minutes to load after turning a unit on. This is frustrating and slows down deployment.

## Testing
To test if the system works, you need at least two units.
Run the `uvdar_stack` though Portainer on both.
Then if you listen to the `/uav##/UWB_UVDAR_Fuser/filtered_poses` topic (or visualize it in RViz), you should be gettin relatively reasonable estimates of the other unit's pose.
In the testing stack, these estimates are in the current UAV's `fcu` frame.

If you have odometry available, it would be better to set the `output_frame` in the `uvdar_core/launch/uwb_uvdar_fuser.launch` to another frame that is world-fixed, in order to filter out ego-motion for better estimate quality.
However, this currently needs a code fix, since the bearing for the purposes of fusing range is now generated assuming that the filter is in the `fcu` frame - see `uvdar_core/src/uwb_uvdar_fuser.cpp:602`.

If the estimate covariance tends to be shaped like a flat "disk" for too long, it implies issues with the vision (UVDAR) part of the system.
If on the other hand it tends to be very elongatte along the line of sight, it implies there is an issue with the UWB part of the system - if the covariance stays in such an elongated state permanently and its mean is very far from the truth, the UWB may not be working at all.
The UWB should be outputting range estimates in the topic `uav##/radar_nodelet/range`, while the vision system should be outputting annotated image positions of the markers in the topics `/$(arg uav_name)/uvdar/blinkers_seen_front` and `/$(arg uav_name)/uvdar/blinkers_seen_back`, and these topics should be outputting something (emtpy messages) even when nothing is seen.
If these input topics are populated, but no pose estimate is being generated, check wheter the `UWB_UVDAR_Fuser` node did not crash with errors pertaining to NaNs or inverting un-invertable matrices.
This is often caused by the camera calibration files not being found, or being somehow corrupted, thus converting the pixel positions into unreasonable bearing covariances
