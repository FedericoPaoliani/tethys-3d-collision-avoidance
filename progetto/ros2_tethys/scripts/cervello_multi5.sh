#!/bin/bash
# Nodi per mondoMulti5.sdf (5 droni). Wrapper su cervello_multi_lanes.sh.
# Uso: ./ros2_tethys/scripts/cervello_multi5.sh [n_ostacoli]
exec "$(dirname "$0")/cervello_multi_lanes.sh" 5 "$@"
