#!/usr/bin/env bash
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-03-11
################################################################

map_name=$1
if [ -z "$map_name" ]; then
    map_name="default"
fi

rosrun hex_map_3d trans_modifier.py $map_name
rosrun hex_map_3d map3d_transform.py $map_name
rosrun hex_map_3d map2d_transform.py $map_name
