# CrazyLander
Automatic landing system for crazyflie

## build
```shell
mkdir build
cd build
cmake ..
make -j4
```

## camera settings
For Aruco detector to perform well, the level of exposure has to be set a low as possible for sharp images.
Adding light in the scene allows lower exposure.

```shell
sudo apt-get install v4l-utils
v4l2-ctl -c exposure_gain=255     #use lower value if noisy
v4l2-ctl -c exposure_auto=1       #disable auto eposure
v4l2-ctl -c exposure_absolute=100 #test to find the lowest working value
```

## ros launch
```shell
roscore
rosrun  aruco_detector aruco_detector_node
rosparam set joy_node/autorepeat_rate 50
rosrun joy joy_node
rosrun crazy_pid crazy_pid_node
rosrun crazy_controller crazy_controller.py
```
