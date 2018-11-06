# Prius Spawn Behavior
The following example in VisualStates demonstrates the Prius Toyota Car Spawn behavior in Gazebo 9 developed through ROS Kinetic. The behavior consists of 1 parent state with 4 child states. The parent state `SpawnPrius` contains 4 child states - `SteerLeft`, `SteerRight`, `MoveForward`, `Stop`.

## Steps to run the example
### Dependencies
We assume that you already installed ROS Kinetic and Gazebo 9 on Ubuntu 16.04 system to be able to test the behaviors. However, if you did not install yet, you can do so following these pages: [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)  [http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

### ROS Package Generation
1. Copy Prius messages, world, description packages from [PriusSpawn](/priusSpawn) and paste it in the ROS Workshop. Also copy the VisualStates prius_spawn.xml file which contains the example behavior.
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
cp -r <path_to_visualstates_examples>/priusSpawn/* .
cd ..
```

2. Copy VisualStates package to the workspace and compile the package and source it.
```
catkin_make
source devel/setup.bash
```

3. Generate the ROS Package of the behavior using the **visualstates**.
```
rosrun visualstates main.py <path_to_ros_workspace>/src/prius_spawn/prius_spawn.xml

```
Generate ROS package using `Actions -> Generate Python` menu.

Recompile the ROS Workspace and you would find a new package prius_spawn in the package list. After compiling source the workspace again.

```
source devel/setup.bash
```
Add the CloverLeaf model to Gazebo models by adding the following line to your environment variables.
```
export GAZEBO_MODEL_PATH=<path_to_PriusSpawnExample>/prius_gazebo/models:$GAZEBO_MODEL_PATH
```
Start the Gazebo simulator and spawn Toyota Prius Car with Prius world, using the toyota prius world launch file
```
roslaunch prius_gazebo prius_world.launch
```
Run our generated ROS Node and visualize Toyota Prius Car Spawn Behavior.
```
rosrun prius_spawn prius_spawn.py --displaygui=true
```
