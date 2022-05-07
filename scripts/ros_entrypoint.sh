#!/bin/bash
set -e

ros_env_setup="/opt/ros/foxy/setup.bash"
ros_noetic_setup="/opt/ros/noetic/setup.bash"

echo "sourcing   $ros_env_setup"
source "$ros_env_setup"


echo "sourcing   $ros_noetic_setup"
source "$ros_noetic_setup"

# ros_workspace_setup="$WORKSPACE_ROOT/install/local_setup.bash"
# echo "sourcing   $ros_workspace_setup"
# source "$ros_workspace_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

exec "$@"

# sudo usermod -aG i2c $USER

#cd /workspace; catkin_make;
