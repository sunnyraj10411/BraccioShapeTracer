# Braccio Shape Tracer


The repository uses Large Language Models (LLMs) to generate coordinates of various shapes that are then traversed using the Braccio robot. The purpose of this repository is to teach students robotic control using LLMs. Most of the code base has been picked up from https://github.com/lots-of-things/braccio_moveit_gazebo. The following updates and changes have been made. 

Update: 
1. The code base was updated to Python 3 from the previous Python 2. 
2. Using the latest ROS version as of March 2024.

Changes: 
1. Added the path tracing code. 
2. Enabled full 360-degree motion of the base. 
3. Made the inverse kinematic solution stable so that solutions are generated for most accessible coordinates. 
4. The camera angle has been adjusted to a top-down view to see the shapes being traced by the robot.   


![Braccio Pick+Drop Demo](doc/triangle1.gif)

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
This script should be run in another terminal. The script will allow you to choose the shape you want to trace. 

```
source devel/setup.bash
rosrun braccio_moveit_gazebo target_object_sim.py
```

## Tracing a pentagon 
The points for tracing the pentagon were generated using GPT3.5 and can be found in the GPT_Code folder. Even though the correct points are generated, they have to be rearranged so that the robot follows an anticlockwise trace.

![Braccio Pick+Drop Demo](doc/pentagon.gif)

