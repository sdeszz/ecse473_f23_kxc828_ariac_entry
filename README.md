# ecse473_f23_kxc828_ariac_entry
# Introduction
Welcome to the ecse473_f23_kxc828_ariac_entry ROS package, designed to interface with the ARIAC (Agile Robotics for Industrial Automation Competition) 2019 environment. This package is structured for participants with basic knowledge in ROS and aims to provide an intuitive approach to start the competition, subscribe to relevant topics, and utilize services effectively within the ARIAC framework.

# Package Structure
launch
launch/second.launch
src
src/node_source.cpp
CMakeLists.txt
package.xml
README.md

# Dependencies
The package depends on the standard ROS packages and any additional dependencies required for ARIAC 2019. Ensure all dependencies are declared in the package.xml file. Use the gitclone command to download the ariac_ws workspace in advance. This workspace can simulate the playing field using the gazebo environment.

# Functionality
Start the Competition:
Use start_competition service to initiate the competition.
Errors in service calls are accompanied by strong error messages.
A detailed message is displayed if the service call is successful but unable to start the service.
Subscribe to Orders:
The package subscribes to the Orders topic to receive order details.
Material Location Service:
Utilizes material_location service to find bins containing parts required for the first product in the first shipment of the first order.
Logical Camera Subscription:
Subscribes to all logical_cameras and stores the information efficiently.
Display Part Location:
Upon receiving an order, the package prints a ROS_WARN[_STREAM] message once, indicating the x, y, and z pose of a part of the correct type in a location identified by the material_location service. The location is provided in the reference frames of the logical_camera and robot arm1_base_link.
Launching the Package

To launch the package, use the following command:

roslaunch osrf_gear sample_environment.launch

This command initializes all nodes and services required for the package's functionality.

Additional Information
For more details on ARIAC 2019, refer to the official ARIAC 2019 documentation.

Conclusion
This package provides a streamlined approach for participating in ARIAC 2019 with ROS. By following the instructions and utilizing the provided functionalities, users can effectively engage in the competition and develop their robotic solutions.
