# Teleoperation under communication loss 

## Setup

Before running the following scripts compile the [CRTK plugin][crtkplug] and add the build path to the `launch.yaml` file.

[crtkplug]: https://github.com/LCSR-CIIS/ambf_crtk_plugin

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


Camera registration repo: https://github.com/jabarragann/dvrk-camera-registration
