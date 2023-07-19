# A simple ROS package of Semi-Global Matching for AirSim
The code was tested with Ubuntu 20.04, ROS Noetic, CUDA 11.4.

The implementation of ``sgm_gpu`` was obtained from [agile_autonomy](https://github.com/uzh-rpg/agile_autonomy).
## How to use
check ``settings.json``file and replace the settings for cameras into your airsim settings.
run airsim simulator and
```rosrun airsim_stereo airsim_stereo```

## Caution
This code is roughly designed for research purposes, so the camera settings are hard-coded in ``airsim_stereo.cpp``.
The implementation of retrieving settings from AirSim is planned for the future.




