# Navion ROS example package 

## Overview
The package provides examples to use the Navion system with ROS. 

## Dependencies

### nav_launch

The Navion system is interfaced with the package ```nav_launch```.

### ps_navi_controller

The PlayStation Navi controller is interfaced with the package ```ps_navi_controller```.

### VideoStreamOpenCV

The camera is interfaced with the features from the package ```video_stream_opencv```: http://wiki.ros.org/video_stream_opencv

## Usage

### Launch files

#### navion_rviz.launch.launch
Uses Navion with a generic RViz interface for research purposes. The control of the field is performed with a magnetic RViz panel. The user can also control the individual currents in the coils.

#### navion_ps_navi.launch
Launches ```navion_rviz.launch``` and interfaces with a PlayStation Navi controller.

#### navion_cam.launch
Interfaces a camera using the VideoStreamOpenCV package.
