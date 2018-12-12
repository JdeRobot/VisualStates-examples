# Prius Obstacle Avoidance Behavior
The following example in VisualStates demonstrates the Prius Toyota Car Obstacle Avoidance behavior in Gazebo 8 developed through ROS Kinetic. The following examples illustrates the Import Functionality present in VisualStates. The prebuilt behavior of ObstacleAvoidance for TurtleBot is imported and updated to be used for Prius Obstacle Avoidance behavior. Also, the child-states developed in PriusSpawn behavior are used to drive the car.

## Steps to run the example
### Dependencies
We assume that you already installed ROS Kinetic and Gazebo 8 on Ubuntu 16.04 system to be able to test the behaviors. However, if you did not install yet, you can do so following these pages: [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)  [http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

### ROS Package Generation
1. Copy Prius messages, world, description packages from [PriusData](/priusData) and paste it in the ROS Workshop. Also clone the VisualStates package and copy the VisualStates prius_obstacle_avoidance.xml file which contains the example behavior.
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone https://github.com/JdeRobot/VisualStates.git
cp -r <path_to_visualstates_examples>/priusData/* .
cp -r <path_to_visualstates_examples>/priusObstacleAvoidance/* .
cd ..
```

2. Compile the package and source it.
```
catkin_make
source devel/setup.bash
```

3. Generate the ROS Package of the behavior using the **visualstates**.
```
rosrun visualstates main.py <path_to_ros_workspace>/src/priusObstacleAvoidance/prius_obstacle_avoidance.xml

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
roslaunch prius_gazebo prius_obstacle_avoid.launch
```
Run our generated ROS Node and visualize Toyota Prius Car Obstacle Avoidance Behavior.
```
rosrun prius_obstacle_avoidance prius_obstacle_avoidance.py --displaygui=true
```
