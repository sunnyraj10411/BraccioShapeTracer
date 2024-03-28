![Braccio Pick+Drop Demo](doc/triangle.gif)

## Install ROS Neotic
```
sudo apt install ros-neotic-desktop-full
```
## Check if ROS and Gazebo is working properly

```
gazebo
gazebo --version
which gzserver
which gzclient
```
## Download Braccio Shape Tracer
```
mkdir BraccioShapeTracer
cd BraccioShapeTracer
mkdir src
cd src
gh repo clone sunnyraj10411/BraccioShapeTracer
```

## Install Braccio Shape Tracer
```
cd ../
source /opt/ros/noetic/setup.bash
go to director 
catkin_init_workspace
catkin_make
```
## Launch Gazebo
```

```

