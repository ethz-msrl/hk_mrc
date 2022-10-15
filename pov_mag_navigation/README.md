# Package for POV magnectic navigation

This package provides point of view (POV) steering functionnalities using magnetic navigation for endoscopic application. 
The key feature is to be able to navigate the endoscope using the endoscopic camera view, and steer the endoscope with motions relative to the image rather than to the eMNS.

## Dependencies

### OpenCVapps

The POV steering requires the optical flow features from the package ```opencv_apps```: http://wiki.ros.org/opencv_apps

## Launch files

*  ```pov_nav_cmag_transnasal.launch```: POV steering of the transnasal endoscope with the CardioMag using a PS navi joystick
*  ```pov_nav_navion_transnasal.launch```: POV steering of the transnasal endoscope with the Navion using a PS navi joystick

## Nodes

*  ```feedback_plot.py```: rudimentary radar plot for the feedback of the steering control and image motion for the POV navigation
*  ```pov_navigation_node.py```: node to perform the POV navigation. Parameters can be tuned via the configuration server.
*  ```ps_navi_controller```: node to publish desired speed in the camera image based on the inputs of a PS navi controller
