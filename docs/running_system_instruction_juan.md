
## Repositories

- [Jack aruco detection](https://github.com/JackHaoyingZhou/aruco_detection)

## Running the system
Terminal 1 - launch robot.

```
roscd dvrk_config_jhu
cd jhu-daVinci
rosrun dvrk_robot dvrk_console_json -j console-SUJ-ECM-MTMR-PSM1-MTML-PSM2-Teleop.json
```

Terminal2 
```
roslaunch dvrk_video decklink_stereo_1280x1024.launch stereo_rig_name:=davinci_endoscope stereo_proc:=True
```

Terminal3 - calibration
```
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 12x10 --square 0.005 right:=/davinci_endoscope/right/image_raw left:=/davinci_endoscope/left/image_raw left_camera:=/davinci_endoscope/left right_camera:=/davinci_endoscope/right
```

Terminal4 - launch aruco detection
```
rosrun aruco_detection aruco_detector.py -c /zedm/zed_node/left -i
```

