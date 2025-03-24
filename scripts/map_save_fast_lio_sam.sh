#!/usr/bin/env bash
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-03-12
################################################################

map_name=$1
if [ -z "$map_name" ]; then
    map_name="default"
fi

if [ ! -d "`rospack find hex_map_3d`/output" ]; then
    mkdir "`rospack find hex_map_3d`/output"
fi

echo "### map name: $map_name ###"
echo "### start to save map ###"
rosservice call /fast_lio_sam/save_map 0.05 "`rospack find hex_map_3d`/output/${map_name}"
