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
source devel/setup.bash
roslaunch braccio_moveit_gazebo rviz_connected_with_gz_using_moveit.launch
```

## Launch tracing script
To be run in another terminal. The script will give you the option to choose the shape that you want to trace. 
```
source devel/setup.bash
rosrun braccio_moveit_gazebo target_object_sim.py
```

## Pentagon tracing

![Braccio Pick+Drop Demo](doc/pentagon.gif)

