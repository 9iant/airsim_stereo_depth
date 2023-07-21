# A simple ROS package of Semi-Global Matching for AirSim
The code was tested with Ubuntu 20.04, ROS Noetic, CUDA 11.4.

The implementation of ``sgm_gpu`` was obtained from [agile_autonomy](https://github.com/uzh-rpg/agile_autonomy).
## How to use
check ``settings.json`` and replace the settings for cameras into your airsim settings.
run airsim simulator and
```roslaunch airsim_stereo airsim_stereo.launch```

![example1](./image/test1.gif)
![example1](./image/test2.gif)
## Caution
This code is roughly designed for research purposes.
Adjust the camera settings in ``airsim_stereo.launch`` to align with your AirSim configurations in ``settings.json``

Check your CUDA version and modify ``CMakeLists.txt``.
usually ``find_package(CUDA REQUIRED)`` will be fine.

## Future work
Retrieving settings from AirSim

