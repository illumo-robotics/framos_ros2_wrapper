# ROS2 Wrapper for FRAMOS D400e camera series based on Intel ROS2 Wrapper v3.2.3

This readme file provides instructions on how to use the D400e camera series with ROS2.

## Supported platforms

This version supports ROS2 Dashing on Ubuntu 18.04, ROS2 Eloquent on Ubuntu 18.04, ROS2 Foxy on Ubuntu 20.04, ROS2 Humble on Ubuntu 22.04.

## Prerequisites

FRAMOS CameraSuite version 4.10.0.0 or higher

Intel� RealSense� SDK with support for D400e camera series version v2.50.10 or higher (see [realsense2_camera release notes](https://github.com/IntelRealSense/realsense-ros/releases))

## Notes

The ROS2 Wrapper for FRAMOS D400e camera series is based on and is very similar to 
ROS Wrapper for FRAMOS D400e camera series.

## Install the ROS2 distribution

### Prerequisites

Make sure that `main`, `universe`, `restricted` and `multiverse` repositories are added

```
sudo add-apt-repository main
sudo add-apt-repository universe
sudo add-apt-repository restricted
sudo add-apt-repository multiverse
sudo apt update
```

#### Install ROS2 Dashing on Ubuntu 18:
Install ROS2 Dashing Desktop package following instructions available at:
[ROS2 Dashing](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html)

```bash
ROS_DISTRO=dashing
```

#### Install ROS2 Eloquent on Ubuntu 18:
Install ROS2 Eloquent Desktop package following instructions available at:
[ROS2 Eloquent](https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html)

```bash
ROS_DISTRO=eloquent
```

#### Install ROS2 Foxy on Ubuntu 20:
Install ROS2 Foxy Desktop package following instructions available at:
[ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

```bash
ROS_DISTRO=foxy
```

#### Install ROS2 Humble on Ubuntu 22:
Install ROS2 Humble Desktop package following instructions available at:
[ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

```bash
ROS_DISTRO=humble
```

### Environment setup
```bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install additional required packages
```
sudo apt install python3-colcon-common-extensions -y
sudo apt install ros-$ROS_DISTRO-diagnostic-updater
sudo apt install libyaml-cpp-dev
```
### Install argcomplete (optional)
#### Dashing/Eloquent
```
sudo apt install -y python3-pip
pip3 install -U argcomplete
```

#### Foxy
```
sudo apt install -y python3-argcomplete
```

### Build Intel� RealSense� ROS2 wrapper from sources
This instructions are based on tutorial "Using colcon to build packages" available at: 
https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/.
Prepare colcon workspace and copy the content of this archive into workspace.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
cp -r /usr/src/librealsense2/wrappers/ros2/. .
cd ~/ros2_ws
```

### Install dependencies:
```bash
sudo apt-get install python3-rosdep -y
sudo rosdep init
rosdep update --include-eol-distros
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
sudo apt purge ros-$ROS_DISTRO-librealsense2 -y
```

## Build ROS2 wrapper package

### Build:
```bash
colcon build --symlink-install
```

### Source (on each new terminal):
```bash
. install/local_setup.bash
```

## Usage Instructions

### Usage Instructions for single camera

Enter the serial numbers or IP addresses of cameras in the `serial_no` or `ip_address` fields in the file `d400e.yaml` to use specific cameras. Serial number has precedence over IP address. If both fields are empty, the first detected camera is used.
Start the camera node in ROS2:
```bash
ros2 launch realsense2_camera d400e_rs_launch.py
```

Launch `rviz` in another terminal

```bash
ros2 run rviz2 rviz2
```

In the `rviz` GUI 
- change the `Fixed Frame` from `map` to `camera_depth_frame`
- click `Add`, select `By topic` and choose desired topic.

To see image in `rviz`, `rviz` Subscription QoS policies must be compatible with Publisher QoS policies (https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/):
To get the Publisher QoS policies, launch in another terminal:
```bash
ros2 topic list
ros2 topic info -v <topic>
```
In example, to get verbose info on `image_rect_raw` topic from depth stream on camera use command:
```bash
ros2 topic info -v /camera/depth/image_rect_raw
```

### Usage instruction for two or more cameras

Enter the serial numbers or IP addresses of cameras in the `serial_no` or `ip_address` fields in the file `d400e_multi_camera.yaml` to use specific cameras.
Modify `d400e_multi_camera.yaml` config file located in `~/ros2_ws/src/realsense2_camera/config`. In the file `d400e_multi_camera.yaml`, create a new `cameraN` entry for each additional camera. Make sure that the prefixes of properties match the camera name. Use the existing `camera1` and `camera2` entries as a reference. Modify `d400e_multi_camera_rs_launch.py` launch file located in `~/ros2_ws/src/realsense2_camera/launch`. Add to `local_parameters` a name for each new camera added. Create `paramsN` for each new camera. Add `d400e_rs_launch.declare_configurable_parameters(paramsN)` for each new camera, and add `IncludeLaunchDescription(...)` for every new `paramsN`. Use existing `d400e_multi_camera_rs_launch.py` as reference.
Rebuild ROS2 wrapper package if modified.

Start the camera nodes in ROS2:
```bash
ros2 launch realsense2_camera d400e_multi_camera_rs_launch.py config_file1:="'src/realsense2_camera/config/d400e_multi_camera.yaml'" config_file2:="'src/realsense2_camera/config/d400e_multi_camera.yaml'"

```

### Available services
Enable : Start/Stop all streaming sensors. Usage example:
Get the service name for enabling/disabling stream
```bash
ros2 service list
```
Call service:
```bash
ros2 service call /camera/enable std_srvs/srv/SetBool "{data: False}"
```

## Device filtering by serial number or/and IP address

Device filtering feature enables applications to connect only to cameras that are specified in the filtering list.
For more details please refer to `d400e_api_extensions.md`

## Setting camera parameters 

### Using config (.yaml) file

Setting camera parameters through a config file is done in the following way (e.g. setting packet_size to 1500 and inter_packet_delay to 39 for stereo module):
```
camera:
  camera:
    ros__parameters:
      stereo_module:
        packet_size: 1500.0
        inter_packet_delay: 39
```

### Using launch file

Setting camera parameters through a launch file is done by adding values to `configurable_parameters` variable. Syntax for dictionary that goes into said variable is as follows:
`{'name': '<parameter_name>', 'default': '<value>', 'description': '<description'}`
e.g. following dictionary sets the exposure value of rgb_camera to 500 leaving description field empty:
`{'name': 'rgb_camera.exposure', 'default': '500', 'description': ''}`
NOTE: If the same parameter is specified in both launch and config files, value in config file has precedence over the one in the launch file (even if the value in config file isn't specified).

### Using `ros2 param` command

For the entire list of parameters, type `ros2 param list`.
To get/set parameters, use:
`ros2 param get <node_name> <parameter_name>`
`ros2 param set <node_name> <parameter_name> <value>`


## Known issues and limitations
