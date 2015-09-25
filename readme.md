Crazyflie AR Detector
=====================

This program is greatly based on the OpenCV3 [detect_marker example](https://github.com/Itseez/opencv_contrib/blob/master/modules/aruco/samples/detect_markers.cpp).

Tested only on Ubuntu Linux. Should work on all Linux distribution and may work
on Mac and Windows. The instuctions are for Ubuntu Linux 14.04 and 15.04.

Detects an AR marker attached under or above the
Crazyflie and sends the X/Y/Z/Yaw position via ZMQ.

This program is used with the webcam-ar branch of the [crazyflie-vision](https://github.com/bitcraze/crazyflie-vision/tree/webcam-ar)
project and the [Crazyflie client](https://github.com/bitcraze/crazyflie-clients-python)
[ZMQ input](https://wiki.bitcraze.io/doc:crazyflie:client:pycfclient:zmq#input_device).

More documentation on the autonomous flight architecture are on the
[Bitcraze wiki](https://wiki.bitcraze.io/doc:crazyflie:vision:index).

Marker
------

This is the marker setup by default (ArUco dictionary 0, Marker 1):

![Marker 1](extra/1.png)

By default the printed dimension should be 35mm. See extra/config.yml for
the configuration

The marker can be taped under a Crazyflie breakout, prototyping deck or motor holder.

Compilation
-----------

The requirements are:
 - libzmq3-dev
 - OpenCV3 compiled with contrib (see bellow for compilation under linux)

It compiles with cmake:

```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

Setting up the camera requires v4l2 tools:
```
sudo apt-get install v4l2ucp v4l-utils
```


Usage
-----

Inspect the configuration file ```extra/config.yml```. The most important is to
setup your camera calibration file. The default setting is for a Logitech C920
modified for longer focus distance. See the OpenCV documentation to calibrate
your camera, we have been using
[calibrate_camera_charuco](https://github.com/Itseez/opencv_contrib/blob/master/modules/aruco/samples/calibrate_camera_charuco.cpp).


Launch the detector with the configuration (-ci 1 opens /dev/video1):
```
$ ./detect_markers -conf ../extra/config.yml -ci 1
```

Setup the camera:
```
$ v4l2ucp /dev/video1
```
 - The exposure time should be set to manual with a value as low as possible. The point is to remove all motion blur.
 - If the camera is autofocus, the focus should be fixed and set for the wanted range.

The camera can be placed on above or bellow the Crazyflie. Set the configuration
"camera_top" to 0 to place the camera bellow. The camera is expected to look straight at the ceiling
or the floor (ie. aligned with the Z axis).

Compile OpenCV3
---------------

Uninstall the Ubuntu openCV package to make sure there is no conflict.

Procedure to compile opencv3 with contrib libs (tested on Ubuntu 14.04):
```
$ git clone https://github.com/Itseez/opencv
$ git clone https://github.com/Itseez/opencv_contrib
$ cd opencv
$ mkdir build
$ cd build
$ cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DBUILD_opencv_legacy=ON ..
$ time make -j6  # This takes about 45min
$ sudo make install
```
