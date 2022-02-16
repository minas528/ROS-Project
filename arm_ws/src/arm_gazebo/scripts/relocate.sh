#!/bin/bash
rostopic pub --once /arm/grip_cmd std_msgs/String grip
rostopic pub --once /arm/angle_cmd geometry_msgs/Quaternion -- 1 2.5 0.3 0
rostopic pub --once /arm/grip_cmd std_msgs/String release
rostopic pub --once /arm/angle_cmd geometry_msgs/Quaternion -- 2 2 0.5 0