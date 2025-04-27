# Tutorial Aerostack2

Aerial Platform interface
- 
Objective be agnostic from the platform and maintain different type of drones

Gazebo platform
- define the simulation in a yaml file, defines drones sensors and other param

State estimation & Motion Controller
-
Wrappere for handling state estimation, we can load different way of odometry and inject in the system

Same for the motion controller

Behaviour
-
Basically extende the action capabilities of ROS2, with this you can change extend pause resume and other stuff.

Plan execution Control
- 
Can be used with behaviour tree, they use the ROS2 wrapper

They have also an interface in python, quite simple to send high-level commands or a JSON with the definition of the various commands

Mission Control
- 
Does not go on the operator but some panel to have the control of the UAV.



--- 
with "raw_odometry" (check that) you send the localization to the drone, tune the pid s


run the estimation with some plugin or other to change the platform


Controller DJI use their controller