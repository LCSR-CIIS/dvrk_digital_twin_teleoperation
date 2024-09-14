
## Repositories
- [GSCAM](https://github.com/hap1961/gscam)
- [Jack aruco detection](https://github.com/JackHaoyingZhou/aruco_detection)
- [ZED mini wrapper](https://github.com/stereolabs/zed-ros-wrapper)
- [CRTK plugin](https://github.com/lcsr-ciis/ambf_crtk_plugin)
- [Registration plugin](https://github.com/LCSR-CIIS/ambf_registration_plugin)
- [Camera Registration](https://github.com/jhu-dvrk/dvrk_camera_registration)

## Instructions:
- [dvrk video devel](https://github.com/jhu-dvrk/dvrk_video/tree/devel)
- [dvrk camera calibration](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Camera-Calibration)


## Camera pipeline

Terminal1 
```
roslaunch dvrk_video decklink_stereo_1280x1024.launch stereo_rig_name:=davinci_endoscope stereo_proc:=True
```

Terminal2 - intrinsic calibration
```
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 12x10 --square 0.005 right:=/davinci_endoscope/right/image_raw left:=/davinci_endoscope/left/image_raw left_camera:=/davinci_endoscope/left right_camera:=/davinci_endoscope/right
```

Terminal3 - hand-eye calibration
```
rosrun dvrk_camera_registration camera_registration.py -p PSM2 -m 0.0147 -c /davinci_endoscope/left
```
Terminal3 - hand-eye results visualization
```
rosrun dvrk_camera_registration vis_gripper_pose.py -p PSM2 -c /davinci_endoscope/left -H PSM2-registration-open-cv.json
```

## Running the robot 
Terminal 1 - launch robot.

```
// For davinci system
roscd dvrk_config_jhu
cd jhu-daVinci
rosrun dvrk_robot dvrk_console_json -j console-SUJ-ECM-MTMR-PSM1-MTML-PSM2-Teleop.json

// OR for dvrk system
roscd dvrk_config_jhu
cd jhu-dVRK
rosrun dvrk_robot dvrk_console_json -j console-MTMR-PSM2-Teleop.json
```

## AR pipeline

Terminal4 - launch aruco detection
```
rosrun aruco_detection aruco_detector.py -c /zedm/zed_node/left -i
rosrun aruco_detection aruco_detector.py -c /davinci_endoscope/left -i
```


## Testing teleoperation of virtual and real PSM arm

Simple debug script
```
python AR_with_full_scene.py  -H ~/temp/ar_test2/PSM2-registration-open-cv.json
```

FULL teleop script
```
python dvrk_teleoperation_ambf.py -m MTMR -H ~/temp/ar_test2/PSM2-registration-open-cv.json
```

## Experiment scripts

Communication loss
```
python com_loss_generator.py
```

Teleoperation in baseline condition
```
python baseline.py -H /home/jbarrag3/ros_workspaces/zed_dvrk_ws/src/davinci_endoscope/calibrations/PSM2-registration-open-cv.json -m MTML -i 0.01
```

Teleoperation in replay condition
```
python replay.py -H /home/jbarrag3/ros_workspaces/zed_dvrk_ws/src/davinci_endoscope/calibrations/PSM2-registration-open-cv.json -m MTML -i 0.01
```