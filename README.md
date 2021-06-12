This works with ROS Foxy

Create a new directory, for example "ws".
Create an src
Introduce the 4 robominer files inside a file named robominer, and leave the ros_ign file aside
You should end up with 2 files: robominer and ros_ign

Build and install:
 cd ~/ws
 colcon build

Gazebo-classic

If you had Gazebo installed when compiling Robominer's packages, Gazebo support should be enabled.
  1. Setup environment variables (the order is important):
   . /usr/share/gazebo/setup.sh
   . ~/ws/install/setup.bash
  
   2.Launch Robominer in a cave (this will take some time to download models):
     ros2 launch robominer_gazebo robominer.launch.py world:=robominer_cave.world
     
  *The previous command has some errors, because it launches the cave without the robot.*
  
  We try the following:
  3. Launch Robominer in an empty world:
     ros2 launch robominer_gazebo robominer.launch.py world:=robominer_empty.world
  
  *It works*
     
Ignition
  1. Setup environment variables (the order is important):
     . ~/ws/install/setup.bash
  2. Launch Robominer in a cave:
     ros2 launch robominer_ignition robominer.launch.py
     
  *It tries to open it but it closes suddenly.*
