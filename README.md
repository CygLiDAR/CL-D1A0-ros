# cld1a0_pcl
cld1a0_pcl is a ROS package, which is designed to visualize 2D/3D distance dataset in real-time.
For more details in the 2D/3D ToF LiDAR, please visit http://www.cygbot.com

## How to build this package
### Preparation
1) Clone this project to your catkin's workspace
2) Build the package
### Set-Up
```bash
roslaunch cld1a0_pcl cld1a0_pcl.launch
```

## Parameters
The parameters for 2D and 3D are 0 and 1 respectively as follows:

2D)
<h1 align="left">
  <img src="screenshots/param_2d.png" width="600"/>
</h1>

3D)
<h1 align="left">
  <img src="screenshots/param_3d.png" width="600"/>
</h1>

### Fixed Frame
```bash
base_laser_link
```
If you don't have the topic on Displays, the param is also mandatory.
### Topic
```bash
/scan
```
### Point Size
Above 5.0 is appropriate in both 2D and 3D.

2D)
<h1 align="center">
  <img src="screenshots/pcl_2d.png" width="800"/>
</h1>

3D)
<h1 align="center">
  <img src="screenshots/pcl_3d.png" width="800"/>
</h1>
