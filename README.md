optical-flow
============

Planar camera position tracking using OpenCV. This program tracks the change is position along the plane perpendicular to the camera's line of sight, given the distance from the camera to the scene (z-depth). The assumption is also made that the scene is relatively flat when compared to the z-depth.

Setup
-----

Before building the project, there are several preprocessor variables that need to be changed to match your camera's setup. The variables can be changed in main.cpp. The variables are listed below:
Z_DEPTH           - The distance from the camera to the scene.
FOCAL_LENGTH      - The focal length of the camera
LENGTH_PER_PIXEL  - The length of one side of a pixel on the image plane. (Can be found using the camera's focal length and view angle.


Build
-----

The project can be built using the XCode Project or the Makefile in the OpticalFlow directory. The Makefile will have to be modified to point to the location where the libopencv_*.so library files are stored. Both methods have headless and headful build configuration. The headful configuration runs with a visual output showing which points are being tracked and their movement in the scene. The headless configuration lacks this visual output.
