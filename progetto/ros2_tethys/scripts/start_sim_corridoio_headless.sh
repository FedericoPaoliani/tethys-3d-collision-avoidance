#!/bin/bash
# Mondo corridoio HEADLESS: mondoCorridoio.sdf (14 ostacoli, corridoio 30×120×20m).
source /opt/ros/humble/setup.bash
source ~/ros2_tethys/install/setup.bash

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/ros2_tethys/install/lrauv_gazebo_plugins/lib:/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins
export LD_LIBRARY_PATH=$HOME/ros2_tethys/install/lrauv_gazebo_plugins/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages
export IGN_GAZEBO_RESOURCE_PATH=$HOME/ros2_tethys/src/lrauv_description/models:$IGN_GAZEBO_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_tethys/src/lrauv_description/models:$GZ_SIM_RESOURCE_PATH
export QT_LOGGING_RULES="qt.qml.connections.warning=false"
export QT_QPA_PLATFORM=xcb
ign gazebo -r -v 3 -s ~/ros2_tethys/worlds/mondoCorridoio.sdf
