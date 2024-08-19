# Simple testing scene to test Augmented reality pipeline.


## Scene 1: Simple scene with cube

```
ambf_simulator --launch_file launch.yaml -l 0 
```
```
python set_simple_scene_with_psm.py
```

## Scene 2: Simple scene with PSM

```
ambf_simulator --launch_file launch.yaml -l 1,2 
```
```
python set_simple_scene_with_psm.py
```

## Scene 3: Simple scene with PSM and custom camera projection
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



## Notes:
- For comparison purposes between real and virtual images use virtual images from the rostopic. A simple way to save images from the rostopic is to use ros tool `image_view` and then right click on the window to save the image.

```
rosrun image_view image_view image:=/ambf/env/cameras/cameraL/ImageData
```
