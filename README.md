# ECSE473 F23 KXC828 ARIAC Entry Package
This repository contains the ecse473_f23_kxc828_ariac_entry package for the ARIAC competition. This ROS package is designed to interface with the ARIAC simulation environment, manage robotic operations, and handle tasks specified for the competition.

# Package Structure
The package includes the following directories and files:

launch/
second.launch: Launch file to start the package.
src/
node_source.cpp: Source code for the ROS nodes.
CMakeLists.txt: CMake configuration file.
package.xml: Package metadata file.
README.md: Documentation file.

# Dependencies
This package requires ROS and the ARIAC 2019 environment. To install them, use following commands:
```
mkdir -p ~/ecse_373_ariac_ws/src
cd ~/ecse_373_ariac_ws/src
git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git
rosdep install --from-paths ecse_373_ariac --ignore-src -r -y
cd ../
catkin_make
source devel/setup.bash
```

Note: Ensure all dependencies are installed before launching the package.

Functionality
The package provides the following functionalities:

Start the Competition: Initiates the ARIAC competition.
Strong error messaging for failed service calls.
Informative messages for unsuccessful attempts.
Subscribe to the Orders Topic: Listens for incoming orders.
Material Location Service: Locates bins with required parts.
Logical Cameras Subscription: Stores and processes camera data.
ROS Warning Message: Prints the part's pose in specified reference frames upon receiving an order.

# Installation and Launching
Clone the repository:
```
git clone https://github.com/sdeszz/ecse473_f23_kxc828_ariac_entry.git
```
Build the package using catkin_make:
```
cd [your_ros_workspace]
catkin_make
```
Launch the package:
```
roslaunch ecse473_f23_kxc828_ariac_entry entry.launch
```

For more information on ARIAC, visit the ARIAC 2019 Documentation.
