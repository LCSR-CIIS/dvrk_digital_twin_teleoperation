# DVRK Digital Twin Teleoperation

The following repository enables: 
* Controlling a real and a digital PSM (running in AMBF) simultaneously using a MTM. 
* Providing AR overlays of the virtual robot on the endoscopic view that is presented to the user.

**Table of Contents**
- [DVRK Digital Twin Teleoperation](#dvrk-digital-twin-teleoperation)
  - [Usage: teleoperation of virtual and real PSM arm](#usage-teleoperation-of-virtual-and-real-psm-arm)
  - [Setup and installation](#setup-and-installation)
    - [0. Install AMBF and Surgical robotics challenge.](#0-install-ambf-and-surgical-robotics-challenge)
    - [1. Compile and setup external plugins](#1-compile-and-setup-external-plugins)
    - [2. Compile internal plugins](#2-compile-internal-plugins)
    - [3. Scene configuration](#3-scene-configuration)
  - [Digital twin calibrations](#digital-twin-calibrations)
    - [Camera calibration and robot-camera registration](#camera-calibration-and-robot-camera-registration)
    - [Register Peg board using PSM tooltip](#register-peg-board-using-psm-tooltip)
      - [Apply the registration result in the scene](#apply-the-registration-result-in-the-scene)
  - [Important repositories](#important-repositories)
  - [Known issues](#known-issues)
  - [Future features](#future-features)
  - [Citation](#citation)

## Usage: teleoperation of virtual and real PSM arm 
The following section describes how to run teleoperation of a virtual and real PSM arm. This instruction assumes that you have previously calibrated your camera and register the camera and the robot.

1. Run real PSM
Run the dVRK console with
```
rosrun dvrk_robot dvrk_console_json -j ~/catkin_ws/src/dvrk/dvrk_config_jhu/jhu-daVinci/console-SUJ-ECM-MTMR-PSM1-MTML-PSM2-Teleop.json -p 0.005
```

2. Start the camera stream with
```
roslaunch dvrk_video decklink_stereo_1280x1024.launch stereo_rig_name:=davinci_endoscope stereo_proc:=True 
```

3. Run virtual PSM
From the desired the root directory of this repository launch AMBF simulation and CRTK interface with:
```
ambf_simulator --launch_file launch.yaml -l 0,1,2,3 --override_max_comm_freq 200 -p 200 -t 1 --conf plugins-config/crtk_config.yaml
```

4. Launch simultaneous teleoperation of real and virtual PSM (requires hand-eye calibration):
```
python dvrk_teleoperation_ambf.py -m MTMR -H ~/temp/ar_test2/PSM2-registration-open-cv.json
```
see [camera_registration_from_SUJ.md](./docs/camera_registration_from_SUJ.md) for more information on how to generate the hand-eye calibration file.


## Setup and installation

### 0. Install AMBF and Surgical robotics challenge.
TODO.

### 1. Compile and setup external plugins 
1. Compile and setup [CRTK plugin][crtkplug], [tf plugin][tfplug] and [registration plugin][regplug].  

```bash
mkdir <your-external-plugin-folder>
cd <your-external-plugin-folder>
git clone https://github.com/LCSR-CIIS/ambf_registration_plugin.git
git clone https://github.com/LCSR-CIIS/ambf_tf_plugin.git
git clone https://github.com/LCSR-CIIS/ambf_crtk_plugin.git

mkdir ambf_crtk_plugin/build ambf_registration_plugin/build ambf_tf_plugin/build


```
Then compile each plugin. 

<details>
  <summary>SSH git clone equivalents</summary>
  <pre><code> 
  cd your-external-plugin-folder
  git clone git@github.com:LCSR-CIIS/ambf_registration_plugin.git
  git clone git@github.com:LCSR-CIIS/ambf_tf_plugin.git
  git clone git@github.com:LCSR-CIIS/ambf_crtk_plugin.git
  </code></pre>
</details>

2. Add a symbolic link to the `external-plugin-folder` inside the `dvrk-ambf-teleoperation` root folder:

```bash
cd <dvrk-ambf-teleoperation-folder>
# Don't use relative links to specify the path to external plugins
ln -s <your-external-plugin-folder>/* plugins/external_plugins/ 
```

Your plugin folder should look like this after performing these steps:
```
plugins/
├── ar_ros_plugin
├── camera_projection_override
├── keyboard_shortcuts_plugin
├── peg_gripper_plugin
├── external_plugins
    ├── ambf_crtk_plugin -> your_external_plugins/ambf_crtk_plugin
    ├── ambf_registration_plugin -> your_external_plugins/ambf_registration_plugin
    └── ambf_tf_plugin ->  your_external_plugins/ambf_tf_plugin
```

### 2. Compile internal plugins

Compile plugins inside the plugins folder with:
```bash
cd plugins
mkdir build
cmake ..
make -j7
```

### 3. Scene configuration
Configure the rostopic names of your endoscopic video in [world_stereo.yaml](./ADF/world/world_stereo.yaml)
```yaml
  left_cam_rostopic: &left_cam_rostopic "/davinci_endoscope/left/image_rect_color"
  left_cam_info_rostopic: &left_cam_info_rostopic "/davinci_endoscope/left/camera_info"

  right_cam_rostopic: &right_cam_rostopic "/davinci_endoscope/right/image_rect_color"
  right_cam_info_rostopic: &right_cam_info_rostopic "/davinci_endoscope/right/camera_info"
```

## Digital twin calibrations

###  Camera calibration and robot-camera registration

TODO.

### Register Peg board using PSM tooltip

First, clone the [Registration Repo](https://github.com/LCSR-CIIS/ambf_registration_plugin) and follow the build instruction. Once you successfully build the repo, use the path to `libregistration_plugin.so` and run the following command:
<path_to_so_file> = `~/ambf_registration_plugin/build/libregistration_plugin.so`
```
ambf_simulator --launch_file launch_registration.yaml -l 0,1,2,3 --conf plugins-config/crtk_config.yaml --registration_config plugins-config/registration_config.yaml
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

#### Apply the registration result in the scene

First, clone the [TF Repo](https://github.com/LCSR-CIIS/ambf_tf_plugin) and follow the build instruction. Once you successfully build the repo, use the path to `libambf_tf_plugin.so` and run the following command:
<path_to_tf_so_file> = `~/ambf_tf_plugin/build/libambf_tf_plugin.so`

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

## Important repositories

* [Camera registration repo][camreg]
* [registration plugin repo][regplug]
* [tf plugin repo][tfplug]
* [crtk plugin repo][crtkplug]

[camreg]: https://github.com/jabarragann/dvrk-camera-registration
[crtkplug]: https://github.com/lcsr-ciis/ambf_crtk_plugin
[tfplug]: https://github.com/LCSR-CIIS/ambf_tf_plugin.git
[regplug]: https://github.com/LCSR-CIIS/ambf_registration_plugin.git

## Known issues 

* Currently the kinematic chain of the virtual PSM and the real PSM are not the same. If both robots are commanded with the same move_jp command the robots will not end up in the same location. Robots are currently being commanded in cartesian space to avoid this issue.
* The AR plugin will generated a segmentation fault if a rosbag recording the videos is launched in the same computer running AMBF.
* The AR plugin will sometimes fails and freeze the video. Check for the line `cout << "INFO! Initilizing rosImageTexture" << endl;` to debug the error.
* Improve building of external plugins with a single build command. Currently, each plugin needs to be built separately. 

## Future features

* Implement trasparency on AMBF objects.
* Migration to ROS2

## Citation 

```
TODO
```