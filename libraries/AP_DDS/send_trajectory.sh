#!/bin/bash

source install/setup.bash
ros2 topic pub /ap/cmd_traj ardupilot_msgs/msg/Trajectory "{
lats: [-35.36142658, -35.35790509, -35.36359994],
lons: [149.16504576, 149.16063726, 149.15248666],
alts: [650.0, 650.0, 650.0]
}" --once

# {lats: [-35.36142658, -35.35790509, -35.36359994], lons: [149.16504576, 149.16063726, 149.15248666], alts: [650.0, 650.0, 650.0]}