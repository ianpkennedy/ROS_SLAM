# Nuturte_control

This package launches the nusim or actual turtlebot, and provides functionality to visualize it in simulation and drive around using an actual robot.

* `roslaunch nuturtle_control start_robot.launch robot:=nusim cmd_src:=<cmd_vel source>` to launch the robot in simulation


* `roslaunch nuturtle_control start_robot.launch robot:=localhost cmd_src:=<cmd_vel source>` to launch the robot while ssh'ed into the turtlebot


* `roslaunch nuturtle_control start_robot.launch robot:=<turtlebotname> cmd_src:=<cmd_vel source> use_rviz:=<true/false>` to launch the robot of name turtlebotname with the cross compilation method, with or without RVIZ. 

* `roslaunch nuturtle_control basic_remote.launch robot:=<turtlebotname> cmd_src:=<cmd_vel source> use_rviz:=<true/false>` to accomplish the same thing as the previous bullet