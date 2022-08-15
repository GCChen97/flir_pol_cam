## flir_pol_cam

This repository provide a driver to publish images from a FLIR gray/color polarization camera with ROS. The codes can be easily modified and integrate in other projects.



### Dependencies

The package has been tested with the following dependencies:

- Ubuntu 18.04/20.04
- ROS [Melodic](wiki.ros.org/melodic/Installation/Ubuntu)/[Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- OpenCV 3/4

- [SpinnakerSDK](https://flir.app.boxcn.net/v/SpinnakerSDK/folder/69083919457)

The package has been tested with the following two models of camera:

- BFS-U3-51S5PC-C: 5.0MP, 75FPS, Sony IMX250MYR, Polari-RGB
- BFS-U3-51S5P-C: 5.0MP, 75FPS, Sony IMX250MZR, Polari-Mono



### Download & Build

```bash
mkdir -p ros_ws/src
cd ros_ws/src
git clone https://github.com/GCChen97/flir_pol_cam.git
cd ..
catkin_make --pkg flir_pol_cam
```

Note that the version of OpenCV used for building should be consistent with the one that ROS uses. Otherwise segmentation fault may occur. You can select OpenCV version by manually modified the `CMakeLists.txt`.



### Camera configuration

Check the configuration file in `configs` folders. Currently, I only add the following parameters.

```txt
[PixelFormat]:
Mono8
[AcquisitionMode]:
Continuous
[Gamma]:
0.5
[ExposureMode]:
Timed
[ExposureTime]:
20000
[AcquisitionFrameRate]:
20
```





### Execute

1. Plug your FLIR polarization camera;
2. Call the launch file.

For grayscale polarization camera, use the following commands:

```bash
roslaunch flir_pol_cam ros_pub.launch
```

For color polarization cameras, use the following commands:

```bash
roslaunch flir_pol_cam ros_pub_color.launch
```

3. Check if it works fine in `rqt_image_view`

```bash
rqt_image_view
```

Then, have fun with polarization imaging :)



### Topic

- "/polcamera/raw": raw image from spinnaker SDK.



### The arrangement of pixels

![arrangement](https://www.flir.eu/globalassets/industrial/discover/machine-vision/imaging-reflective-surfaces-sonys-first-polarized-sensor/myr.jpg)

The image above shows the arrangement of pixels in a raw polarization color image. Note that the bayer color filters only exist in the color model.



### Reference

- `/<path-of-spinnaker-SDK-installed>/src/Polarization/Polarization.cpp` in the example codes of spinnaker SDK

- spinnaker_sdk_camera_driver

  https://github.com/neufieldrobotics/spinnaker_sdk_camera_driver

- Polarized sensor

  https://www.flir.eu/discover/iis/machine-vision/imaging-reflective-surfaces-sonys-first-polarized-sensor/



### License

This project is licensed under the MIT License.
