# Augmented reality pipeline tests.

## Simple tests

### Scene 1: Simple scene with cube

```
ambf_simulator --launch_file launch.yaml -l 0 
```
```
python set_simple_scene_with_psm.py
```

### Scene 2: Simple scene with PSM

```
ambf_simulator --launch_file launch.yaml -l 1,2 
```
```
python set_simple_scene_with_psm.py
```

### Scene 3: Simple scene with PSM and custom camera projection
1. First compile the camera projection override
```
cd <root-of-dvrk-teleoperation>/plugins/
mkdir build && cd build
cmake ..
make 
```
2. Set projection matrix in [custom_camera_projection.yaml](./ADF/custom_camera_projection.yaml).
3. Run simulation
```
ambf_simulator --launch_file launch.yaml -l 1,2,3
```

## FULL AR pipelines
 
Before running the AR pipelines make sure to compile the `camera_projection_override` and `ros_ar` plugins.

### Scene 4: Simple AR/AMBF pipeline with Cube 

1. Launch ZED mini camera and AMBF:
```
roslaunch zed_wrapper zedm.launch
ambf_simulator --launch_file launch.yaml -l 0,4
```
2. Launch arUco detection
```
rosrun aruco_detection aruco_detector.py -c /zedm/zed_node/left -i
```
3. Launch script to control camera position based on marker pose
```
python zed_m_ar_example.py
```
4. Launch blending node 
```
python blending_no_ral.py
```

### Scene 5: Simple AR/AMBF pipeline with  PSM
1. Calculate projection matrix.
2. Run AMBF with PSM
```
ambf_simulator --launch_file launch.yaml -l 1,2,5
```
4. Run AR pipeline
```
python zed_m_psm_ar_example.py -H /home/juan95/Downloads/davinci_ar_data/endoscope_demo/PSM2-registration-open-cv.json
```

## Notes:
- To compare real and virtual images use virtual images from the rostopic. A simple way to save images from the rostopic is to use ros tool `image_view` and then right click on the window to save the image.

```
rosrun image_view image_view image:=/ambf/env/cameras/cameraL/ImageData
```
