#!/bin/bash
# Mondo a corsie parallele mondoMulti5.sdf — 5 droni, vasca 60×120×20 m,
# 22 ostacoli cilindrici, ogni drone con il proprio target.
source /opt/ros/humble/setup.bash
source ~/ros2_tethys/install/setup.bash

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/ros2_tethys/install/lrauv_gazebo_plugins/lib:/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins
export LD_LIBRARY_PATH=$HOME/ros2_tethys/install/lrauv_gazebo_plugins/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages
export IGN_GAZEBO_RESOURCE_PATH=$HOME/ros2_tethys/src/lrauv_description/models:$IGN_GAZEBO_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_tethys/src/lrauv_description/models:$GZ_SIM_RESOURCE_PATH
export QT_LOGGING_RULES="qt.qml.connections.warning=false"
export QT_QPA_PLATFORM=xcb
# Telecamera agganciata al drone con offset fisso (4, 0, 0.7).
# In background e PRIMA di "ign gazebo": lo script attende che la GUI esponga
# /gui/follow, e dopo "ign gazebo" non partirebbe fino alla chiusura della simulazione.
"$(dirname "$0")/segui_drone.sh" tethys_0 4 0 0.7 &

# MONDO=percorso/mondo.sdf sceglie un altro mondo (es. un livello di difficolta').
ign gazebo -v 3 --gui-config ~/ros2_tethys/gui.config "${MONDO:-$HOME/ros2_tethys/worlds/mondoMulti5.sdf}"
# NOTA: senza -r → parte IN PAUSA. Avvia ponte e cervello, poi premi ▶ nella GUI.
