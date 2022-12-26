# BIME Final Project - Software
[TOC]
###### tags: `Mechatronics`

# ROS
![](https://i.imgur.com/ATumWhF.png)
## Install
### Ubuntu
* [**ubuntu 20.04 ROS Noetic**](https://wiki.ros.org/ROS/Installation/TwoLineInstall/)
* [**Install ROS on Ubuntu**](https://wiki.ros.org/noetic/Installation/Ubuntu)
    * `wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh`
### RPi & ROS from Source
* [**How to Install ROS Noetic on Raspberry Pi 4**](https://varhowto.com/install-ros-noetic-raspberry-pi-4/)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
sudo rosdep init
cat /etc/ros/rosdep/sources.list.d/20-default.list
rosdep update

mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
rosinstall_generator ros_comm --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall
wstool init src noetic-ros_comm-wet.rosinstall
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster

sudo dphys-swapfile swapoff
sudo vim /etc/dphys-swapfile // CONF_SWAPSIZE=1024
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
free -m

sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
source /opt/ros/noetic/setup.bash
```
* [**Installing ROS Kinetic on the Raspberry Pi**](https://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi) - This is the official version for Kinetic. Has "Maintaining a Source Checkout"
```
cd ~/ros_catkin_ws
rosinstall_generator ros_comm ros_control joystick_drivers --rosdistro noetic --deps --wet-only --tar > noetic-custom_ros.rosinstall
rosinstall_generator ros_comm turtlesim usb_cam --rosdistro noetic --deps --wet-only --tar > noetic-custom_ros.rosinstall

wstool merge -t src noetic-custom_ros.rosinstall
wstool update -t src

rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
```
### Packages
* `sudo apt-get install ros-$(rosversion -d)-`
* `sudo apt-get install ros-$(rosversion -d)-turtlesim`

### Issues
#### INSTALL ROSDEP
* 建立 package 遇到問題
* ![](https://i.imgur.com/oX87L8T.png)

#### Build only the pkg
![](https://i.imgur.com/Xy2B0ad.png)
https://answers.ros.org/question/54178/how-to-build-just-one-package-using-catkin_make/
`sudo src/catkin/bin/catkin_make_isolated --only-pkg-with-deps bime_final --merge -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3`

#### Can't Execute ROSRUN after source devel/setup.bash
[Ros commands no longer working after source catkin_ws/devel/setup.bash](https://answers.ros.org/question/349276/ros-commands-no-longer-working-after-source-catkin_wsdevelsetupbash/)
* `source /opt/ros/melodic/setup.bash` and then `source devel/setup.bash --extend`

* `sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3`
    * would repair, but very slow on RPi (Seems recompile whole ROS system)

#### catkin_make vs catkin_make_isolated
* [catkin_make vs catkin_make_isolated, which is preferred?](https://answers.ros.org/question/320613/catkin_make-vs-catkin_make_isolated-which-is-preferred/)
![](https://i.imgur.com/2IfxJen.png)

## Connect Master
* [**Connecting Raspberry Pi with PC over LAN**](https://razbotics.wordpress.com/2018/01/23/ros-distributed-systems/)

## Components
### DHT11
[DHT11 introudction](https://blog.csdn.net/wgj99991111/article/details/53749144)
![](https://i.imgur.com/JumtdRG.png)

### OpenCV / Camera
* [Ball Track](https://github.com/tizianofiorenzani/ros_tutorials/tree/master/opencv)
* [CV Camera Tourial](https://www.theconstructsim.com/how-to-install-a-usb-camera-in-turtlebot3/)
* [opencv turtlebot2-tutorials](https://dabit-industries.github.io/turtlebot2-tutorials/14b-OpenCV2_Python.html)
[ROS Developers LIVE Class #86: How to use OpenCV with ROS](https://www.youtube.com/watch?v=0C0gOsLoP9k)
* [cv camera](https://www.theconstructsim.com/how-to-install-a-usb-camera-in-turtlebot3/)
* [cv tracking object](https://github.com/behnamasadi/OpenCVProjects)
* [ROS Tutorial 001: Remotely connecting to webcam using Raspberry Pi 3 and ROS](https://github.com/yev-d/tutos/blob/master/ROS/Tutorial-001/Remotely-connecting-to-webcam-using-Raspberry-Pi-3-and-ROS.md)
* [How to use USB camera with ROS on Raspberry Pi - image_view and web streaming](https://roboticsweekends.blogspot.com/2017/12/how-to-use-usb-camera-with-ros-on.html)
* [rosmaster connect](https://hollyqood.wordpress.com/2017/05/17/ros%E4%B8%8A%E5%90%8C%E6%99%82%E4%BD%BF%E7%94%A8%E5%A4%9A%E5%80%8B%E7%92%B0%E5%A2%83%E9%80%A3%E6%8E%A5masterroscore/) error: cannot connect 
* [**How to use a USB camera with ROS on the Raspberry Pi or BeagleBone Blue - for streaming video to a large computer**](https://sudonull.com/post/14791-How-to-use-a-USB-camera-with-ROS-on-the-Raspberry-Pi-or-BeagleBone-Blue-for-streaming-video-to-a-lar)
* [usb_cam Package Summary](http://wiki.ros.org/usb_cam#usb_cam_node)
* [camera_calibration set_camera_info Service not found](https://answers.ros.org/question/200154/camera_calibration-set_camera_info-service-not-found/)
#### Issues
* [image_view-2] process has died [pid 2280, exit code -11, cmd /opt/ros/noetic/lib/image_view/image_view image:=/usb_cam/image_raw __name:=image_view __log:=/home/ubuntu/.ros/log/ac8e131c-5993-11ed-ba8b-c538adf0cb0e/image_view-2.log].
log file: /home/ubuntu/.ros/log/ac8e131c-5993-11ed-ba8b-c538adf0cb0e/image_view-2*.log
    * [usb cam error](https://www.twblogs.net/a/5c0a98e1bd9eee6fb37bccd7)
* [usb cam service](https://answers.ros.org/question/200154/camera_calibration-set_camera_info-service-not-found/)

##### USB_CAM Launch
![](https://i.imgur.com/in5W8kJ.png)
* Can be ignore on RPi. only need image-view on VM

##### Webcam Issue
* [**Ubuntu Community Webcam**](https://help.ubuntu.com/community/Webcam)
```
sudo usermod -a -G video $LOGNAME 
sudo apt install v4l-utils
sudo apt install ffmpeg
ffplay -f video4linux2 -input_format mjpeg -framerate 5 -video_size 640*480 /dev/video0
```
![](https://i.imgur.com/5QV83pF.png)

```
Driver Info:                    
        Driver name      : uvcvideo                                     
        Card type        : UVC Camera (046d:081b)
        Bus info         : usb-0000:02:03.0-1           
        Driver version   : 5.15.64
        Capabilities     : 0x84a00001
                Video Capture                                           
                Metadata Capture                                        
                Streaming
                Extended Pix Format                                                                                                              
                Device Capabilities                                                                                                              
        Device Caps      : 0x04200001
                Video Capture                                           
                Streaming                                               
                Extended Pix Format
Media Driver Info:                                                                                                                               
        Driver name      : uvcvideo                                                                                                              
        Model            : UVC Camera (046d:081b)                                                                                                
        Serial           : 9D37D020                                     
        Bus info         : usb-0000:02:03.0-1                                                                                                    
        Media version    : 5.15.64                                                                                                               
        Hardware revision: 0x00000012 (18) 
        Driver version   : 5.15.64                                      
Interface Info:                                                         
        ID               : 0x03000002                                                                                                            
        Type             : V4L Video                                                                                                             
Entity Info:                                                                                                                                     
        ID               : 0x00000001 (1)                                                                                                        
        Name             : UVC Camera (046d:081b)
        Function         : V4L2 I/O                                     
        Flags         : default                                                                                                                  
        Pad 0x01000007   : 0: Sink                                      
          Link 0x02000019: from remote pad 0x100000a of entity 'Extension 4': Data, Enabled, Immutable
Priority: 2
Video input : 0 (Camera 1: ok)
Format Video Capture:
        Width/Height      : 640/480
        Pixel Format      : 'MJPG' (Motion-JPEG)
        Field             : None
        Bytes per Line    : 0
        Size Image        : 614400
        Colorspace        : sRGB
        Transfer Function : Rec. 709
        YCbCr/HSV Encoding: ITU-R 601
        Quantization      : Default (maps to Full Range)
        Flags             : 
Crop Capability Video Capture:
        Bounds      : Left 0, Top 0, Width 640, Height 480
        Default     : Left 0, Top 0, Width 640, Height 480
        Pixel Aspect: 1/1
Selection Video Capture: crop_default, Left 0, Top 0, Width 640, Height 480, Flags: 
Selection Video Capture: crop_bounds, Left 0, Top 0, Width 640, Height 480, Flags: 
Streaming Parameters Video Capture:
        Capabilities     : timeperframe
        Frames per second: 5.000 (5/1)
        Read buffers     : 0
                     brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=128
                       contrast 0x00980901 (int)    : min=0 max=255 step=1 default=32 value=32
                     saturation 0x00980902 (int)    : min=0 max=255 step=1 default=32 value=32
 white_balance_temperature_auto 0x0098090c (bool)   : default=1 value=1
                           gain 0x00980913 (int)    : min=0 max=255 step=1 default=64 value=1
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=2 value=2
                                0: Disabled
                                1: 50 Hz
                                2: 60 Hz
      white_balance_temperature 0x0098091a (int)    : min=0 max=10000 step=10 default=4000 value=1070 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=255 step=1 default=24 value=24
         backlight_compensation 0x0098091c (int)    : min=0 max=1 step=1 default=0 value=0
                  exposure_auto 0x009a0901 (menu)   : min=0 max=3 default=3 value=3
                                1: Manual Mode
                                3: Aperture Priority Mode
              exposure_absolute 0x009a0902 (int)    : min=1 max=10000 step=1 default=166 value=656 flags=inactive
         exposure_auto_priority 0x009a0903 (bool)   : default=0 value=1
```
* fixed: need to set the USB port to 3.1
![](https://i.imgur.com/E9kv6c9.png)
![](https://i.imgur.com/86IxPJM.png)

# UbiquityRobotics (Discard)
* https://learn.ubiquityrobotics.com/noetic_pi_image_downloads
* Strange behavior - discard
## rpi wifi connect
[wifi connect](https://learn.ubiquityrobotics.com/noetic_quick_start_microsd)
[wifi connect](https://learn.ubiquityrobotics.com/noetic_quick_start_connecting)
```
pifi list seen
sudo pifi add EELAB-Teach-5G bime0307
sudo pifi add Xiaomi5G 83662290
sudo pifi add Xiaomi5G ntubime405
sudo pifi add BBLAB_New ntubime405 
sudo reboot
```

## Issues
* ![](https://i.imgur.com/tlOkOPk.png)
    




