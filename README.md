# Turtlebot_Walker

## Overview of the Project

The package is to demonstrate the working of a turtlebot robot for collison avoidance in an environment with obstacles and enclosed on all for directions with walls.

## License

The License used for this project is the MIT license.

## Dependencies and Assumptions

The depedencies of the project are 
ROS Kinetic.
Ubuntu 16.04.
catkin.
Gazebo 7.0.
turtlebot_gazebo (Assumed to be installed).
It is also assumed that are no packages in with the same name in the workspace.

## ROS package Dependencies

The ROS package dependencies are:
roscpp
std_msgs
geometry_msgs
sensor_msgs

## Build and Install package

```
cd
#Create your workspace
mkdir -p catkin_ws
cd catkin_ws
#Create source folder inside the workspace
mkdir -p src
#Initialize the workspace
catkin_make
cd src
#Clone the Package into the source folder
git clone https://github.com/Charan-Karthikeyan/turtlebot_walker.git
cd ..
#Building the workspace
catkin_make
```

## Executiong the Package

### To create the custom world file

Open a terminal and run 
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
This command opens a gazebo file with the pre-bulit map that comes with the turtlebot package
We can edit and insert all the walls and objects that we need and save them to the required folder
as a world file. This world file is then sourced into the launch file.
The custom world file designed is shown in the image below
</p>
<p align="center">
<img src="/images/world.png">
</p>
</p>

### Running the Package

Open a terminal and run the command

```
sudo gedit ~/.bashrc
```

Add the following lines to source the workspace
```
source ./catkin_ws/devel/setup.bash
```
Save the file and exit.Then open a terminal and run source the bashrc file
```
source ~/.bashrc
```
Open a new terminal tab and run roscore
```
roscore
```
Then in an unused terminal
```
roslaunch turtlebot_walker turtlebot_walker.launch 
```
or
```
roslaunch turtlebot_walker turtlebot_walker.launch bagrecord:=1
```
The command lauches the package and runs them and records the nodes in the package to a bag file.
The execution of the package is shown in the image below. The value of bagrecord is optional,
the value of the argument is always set to be true to record the bag files and can be turned off
by setting the value of bagrecord to 0. 
The recorded bag files are stored in the rosbag folder in the package file.
</p>
<p align="center">
<img src="/images/final.png">
</p>
</p>

## Cpplint and Cppcheck

To run the cppcheck run the below commands from inside the turtlebot_walker package
```
cppcheck --enable=all --std=c++11 -I ../../devel/include/ -I ../../../../../../../opt/ros/kinetic/include/ -I ../../../../../../../usr/include/ --check-config --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```
To run the cpplint check run the below command from inside the turtlebot_walker package
```
cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )
```
The output from the commands can be seen inside the results folder in the package

## Adding Tag to the branch

To add the tag name for the branch type this inside the package in the terminal.
```
git tag -a Week12_HW_Release -m "week12 tag"
```
To check the tag. Type the following command into the terminal
```
git tag
```
this will show all the tags for the projects.

## Merge with the master branch
To merge the branch into the master branch type the following command into the terminal.
```
git checkout master
```
This commands switches the current branch to master. Then type in 
```
git merge Week12_HW
```
This command merges the branch with the master branch and tries to automatically correct the conflicts. If it is not able to then we have to manually clean the conflicts and push to the repository.

