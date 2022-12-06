# ROS PKG - bime_final_rpi
## Cheatsheet
* Compile this package on RPi
```
sudo src/catkin/bin/catkin_make_isolated --only-pkg-with-deps bime_final_rpi --merge -j4 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Basic Information
### Dependence
* [USB_CAM](https://index.ros.org/p/usb_cam/github-ros-drivers-usb_cam/)

## Component
### Mecanum wheel
![Move Direction](https://user-images.githubusercontent.com/11520473/205495126-5cff45d6-f18e-455f-a3ef-0987e93e8ad0.png)

## Reference
[Raspberry Pi - PWM](https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/)


