# Teleoperation under communication loss 

temporal command
```
ambf_simulator --launch_file launch.yaml -l 1,4,5 --override_max_comm_freq 200 -p 200 -t 1 --conf crtk_config.yaml
```
## Setup

Before running the following scripts compile the [CRTK plugin][crtkplug] and add the build path to the `launch.yaml` file.

[crtkplug]: https://github.com/lcsr-ciis/ambf_crtk_plugin

## Running teleoperation of a virtual and real PSM arm.

From the desired `surgical_robotics_challenge` directory, to launch AMBF simulation and CRTK interface:
```
ambf_simulator --launch_file launch.yaml -l 0,1,4,5,6 --override_max_comm_freq 200 -p 200 -t 1 --conf ~/dvrk_ambf_teleoperation/crtk_config.yaml
```

To run the dVRK console:
```
rosrun dvrk_robot dvrk_console_json -j ~/catkin_ws/src/dvrk/dvrk_config_jhu/jhu-daVinci/console-SUJ-ECM-MTMR-PSM1-MTML-PSM2-Teleop.json -p 0.005
```

To start the camera:
```
roslaunch dvrk_video jhu_daVinci_video.launch 
```

Rectify:
```
ROS_NAMESPACE=/jhu_daVinci/decklink/ rosrun stereo_image_proc stereo_image_proc
```

## Register Peg board using PSM tooltip
First, clone the [Registration Repo](https://github.com/LCSR-CIIS/ambf_registration_plugin) and follow the build instruction. Once you successfully build the repo, use the path to `libregistration_plugin.so` and run the following command:
<path_to_so_file> = `~/ambf_registration_plugin/build/libregistration_plugin.so`
```
ambf_simulator --launch_file launch.yaml -l 2,4,6 --plugins <path_to_so_file> --registration_config registration_config.yaml
```

Once, the registration pipeline is open, Press `[Ctrl + 1]` to activate pin-base registration mode. Press `[Ctrl + 9]` to store the points.
[Caution] Sampling order matters!! Make sure to sample the points in the same order as the points in `registration_config.yaml`.
The calibration results will be printed in the terminal. Copy and paste the results in the ADF: 
```bash
position: {x: 0.0, y:0.0, z:0.0}
orientation: {r: 0.0, p: 0.0, y:0.0}
```


Camera registration repo: https://github.com/jabarragann/dvrk-camera-registration

Registration repo: https://github.com/LCSR-CIIS/ambf_registration_plugin