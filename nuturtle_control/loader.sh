#!/usr/bin/env bash
export ROS_MASTER_URI=http://<your_hostname_here>:11311
source /home/msr/install/setup.bash
exec "$@"