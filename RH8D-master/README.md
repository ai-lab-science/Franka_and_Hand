# RH8
Robot Hand from Seed Robotics RH8 with 19DoF

Quick steps:

1. Follow the steps from seed robotics e-manual: http://kb.seedrobotics.com/doku.php?id=apisandframeworks:ros
2. Or copy the "seed_robotics_ws" in case it gets fully uploaded here

Important notice:
When running ROS noetic, you have to build the Dynamixel_motor package from source (download from github und build in the workspace)
when testing the provided tutorial examples, you will encounter problems with the python version thats delivered with ROS noetic
we provided the working changes in the "seed_robotics_ws"

3. To start the full rh8d hand, launch the "start_rh8d.launch" script 
4. this also starts the node, that builds the connection between the sensorglove and the rh8d hand

Troubleshooting:
- when the publishing or subscribing of the sensorglove or the hand is suffering a high and inconsistent latency, try restarting the router and recheck the ip address of the roscore
