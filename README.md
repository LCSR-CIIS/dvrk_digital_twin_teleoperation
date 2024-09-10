# Teleoperation under communication loss 

## Setup

Before running the following scripts compile the [CRTK plugin][crtkplug] and add the build path to the `launch.yaml` file. You also need the plugins in this repository.

[crtkplug]: https://github.com/lcsr-ciis/ambf_crtk_plugin

## Running teleoperation of a virtual and real PSM arm.

### Virtual PSM
From the desired the root directory of this repository launch AMBF simulation and CRTK interface with:
```
ambf_simulator --launch_file launch.yaml -l 0,1,4,5,6 --override_max_comm_freq 200 -p 200 -t 1 --conf plugins-config/crtk_config.yaml
```

### Real PSM
To run the dVRK console:
```
rosrun dvrk_robot dvrk_console_json -j ~/catkin_ws/src/dvrk/dvrk_config_jhu/jhu-daVinci/console-SUJ-ECM-MTMR-PSM1-MTML-PSM2-Teleop.json -p 0.005
```

To start the camera:
```
roslaunch dvrk_video decklink_stereo_1280x1024.launch stereo_rig_name:=davinci_endoscope stereo_proc:=True 
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

## Important repositories:

* Camera registration repo: https://github.com/jabarragann/dvrk-camera-registration
* Registration repo: https://github.com/LCSR-CIIS/ambf_registration_plugin