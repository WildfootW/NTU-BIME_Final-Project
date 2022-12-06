# RPi Environments
## WPA-Supplicant
* Modify `wpa_supplicant.conf` content to meet AP requirnments.
* Rename `wpa_supplicant.conf` to `wpa_supplicant.conf.SSID.ignore` to prevent from leaking credentials.

## .bashrc
### w-vm-noetic
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311/
# Casa pendente
export ROS_HOSTNAME=xxx.xxx.xxx.131
export ROS_IP=xxx.xxx.xxx.131

# EELab
#export ROS_HOSTNAME=xxx.xxx.xxx.49
#export ROS_IP=xxx.xxx.xxx.49
```

### w-pi-noetic
```
source /opt/ros/noetic/setup.bash
source ~/ros_catkin_ws/devel_isolated/setup.bash --extend

# Casa pendente
export ROS_MASTER_URI=http://xxx.xxx.xxx.131:11311/
export ROS_HOSTNAME=xxx.xxx.xxx.133
export ROS_IP=xxx.xxx.xxx.133

# EELab
#export ROS_MASTER_URI=http://xxx.xxx.xxx.49:11311/
#export ROS_HOSTNAME=xxx.xxx.xxx.84
#export ROS_IP=xxx.xxx.xxx.84
```
