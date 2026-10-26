#!/bin/bash
# Bridge per mondoMulti5.sdf (5 droni). Wrapper su ponte_multi_lanes.sh.
exec "$(dirname "$0")/ponte_multi_lanes.sh" 5
