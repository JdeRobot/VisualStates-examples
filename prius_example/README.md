# Prius Example
The following example in VisualStates demonstrates the Prius Toyota Car Examples in Gazebo 8 developed through ROS Kinetic. The following examples illustrates the several functionalities of the tool. The example starts with prius off-road and further through stop sign the car stops for a particular amount of time. Further takes a turn and gets on highway.

## Steps to run the example
### Dependencies
We assume that you already installed ROS Kinetic and Gazebo 8 on Ubuntu 16.04 system to be able to test the behaviors. However, if you did not install yet, you can do so following these pages: [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)  [http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

### ROS Package Generation
1. Copy Prius messages, world, description packages from [PriusData](/prius) and paste it in the ROS Workshop. Also clone the VisualStates package and copy the VisualStates prius_example.xml file which contains the example behavior.
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone https://github.com/JdeRobot/VisualStates.git
cp -r <path_to_visualstates_examples>/prius/* .
cp -r <path_to_visualstates_examples>/prius_example/* .
cd ..
```

2. The example also requires the darknet_ros package which would run Yolo v3 on darknet compatible with ROS. Further compile all the ROS packages and source the workspace.
```
cd src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
cd ..
catkin_make
source devel/setup.bash
```

3. Generate the ROS Package of the behavior using the **visualstates**.
```
rosrun visualstates main.py <path_to_ros_workspace>/src/prius_example/prius_example.xml

```
Generate ROS package using `Actions -> Generate Python` menu.

Recompile the ROS Workspace and you would find a new package prius_example in the package list. After compiling source the workspace again.

```
source devel/setup.bash
```
Add the Prius Example Environment model to Gazebo models by adding the following line to your environment variables.
```
export GAZEBO_MODEL_PATH=<path_to_PriusSpawnExample>/prius_gazebo/models:$GAZEBO_MODEL_PATH
```
Start the Gazebo simulator and spawn Toyota Prius Car with Prius world, using the toyota prius world launch file
```
roslaunch prius_gazebo prius_example.launch
```
Run our generated ROS Node and visualize Toyota Prius Car Example Behavior.
```
rosrun prius_example prius_example.py --displaygui=true
```
