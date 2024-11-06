# Teleoperation under communication loss 

- [Teleoperation under communication loss](#teleoperation-under-communication-loss)
  - [Usage](#usage)
    - [Virtual PSM](#virtual-psm)
    - [Real PSM](#real-psm)
  - [Register Peg board using PSM tooltip](#register-peg-board-using-psm-tooltip)
    - [Apply the registration result in the scene](#apply-the-registration-result-in-the-scene)
  - [Setup and installation](#setup-and-installation)
    - [1. Plugin configuration](#1-plugin-configuration)
    - [2. Scene configuration](#2-scene-configuration)
  - [Important repositories](#important-repositories)
  - [Future improvements](#future-improvements)

## Usage
The following section describes how to run teleoperation of a virtual and real PSM arm.

### Virtual PSM
From the desired the root directory of this repository launch AMBF simulation and CRTK interface with:
```
ambf_simulator --launch_file launch.yaml -l 1,4,5,6 --override_max_comm_freq 200 -p 200 -t 1 --conf plugins-config/crtk_config.yaml
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
ambf_simulator --launch_file launch.yaml -l 1,6,7 --plugins <path_to_so_file> --registration_config plugins-config/registration_config.yaml
```
[Alternative] Add the following content in your `launch.yaml`.
```
{
   path: <path_to_registration_build_folder>, #THIS IS ENV SPECIFIC 
   name: registration_plugin,
   filename: libregistration_plugin.so
 }
```

Once, the registration pipeline is open, Press `[Ctrl + 3]` to activate pin-base registration mode. Press `[Ctrl + 9]` to store the points.
[Caution] Sampling order matters!! Make sure to sample the points in the same order as the points in `registration_config.yaml`.
The calibration results will be printed in the terminal. Copy and paste the results in the ADF: 
```bash
position: {x: 0.0, y: 0.0, z: 0.0}
orientation: {r: 0.0, p: 0.0, y: 0.0}
```

### Apply the registration result in the scene
First, clone the [TF Repo](https://github.com/LCSR-CIIS/ambf_tf_plugin) and follow the build instruction. Once you successfully build the repo, use the path to `libambf_tf_plugin.so` and run the following command:
<path_to_tf_so_file> = `~/ambf_tf_plugin/build/libambf_tf_plugin.so`
```
Change the configuration file, `plugins-config/tf_PegBoard.yaml` by copy and pasting the result from the previous registration section.
Lastly, in order to apply this registrarion result, add the following options when running your ambf_simulator:
```bash
--plugins <path_to_tf_so_file>  --tf_list plugins-config/tf_PegBoard.yaml
```
[Alternative] Add the following content in your `launch.yaml`.
```
{
   path: <path_to_tf_build_folder>, #THIS IS ENV SPECIFIC 
   name: ambf_tf_plugin,
   filename: libambf_tf_plugin.so
 }
```
## Setup and installation

### 1. Plugin configuration
1. Before running the following scripts compile the [CRTK plugin][crtkplug], [tf plugin][tfplug] and [registration plugin][regplug]. Add the build path to the `launch.yaml` file.  
2. Compile plugins inside the plugins folder with:
```bash
cd plugins
mkdir build
cmake ..
make -j7
```

### 2. Scene configuration
TODO.


## Important repositories

* [Camera registration repo][camreg]
* [Registration repo][regplug]
* [tf plugin][tfplug]
* [crtk plugin][crtkplug]

[camreg]: https://github.com/jabarragann/dvrk-camera-registration
[crtkplug]: https://github.com/lcsr-ciis/ambf_crtk_plugin
[tfplug]: https://github.com/LCSR-CIIS/ambf_tf_plugin.git
[regplug]: https://github.com/LCSR-CIIS/ambf_registration_plugin.git

## Future improvements

TODO