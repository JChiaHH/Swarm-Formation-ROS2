#!/bin/bash
# Verify obstacle detection and SLAM visualization for all 7 drones

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0

echo "============================================"
echo " SLAM / Obstacle Detection Verification"
echo "============================================"
echo ""

# 1. Global map
echo "=== Global Obstacle Map ==="
if timeout 3 ros2 topic echo /map_generator/global_cloud --once &>/dev/null; then
    echo -e "${GREEN}[PASS]${NC} /map_generator/global_cloud is publishing"
    ((PASS++))
else
    echo -e "${RED}[FAIL]${NC} /map_generator/global_cloud not publishing"
    ((FAIL++))
fi
echo ""

# 2. Per-drone local point clouds
echo "=== Per-Drone Local Point Clouds ==="
for i in $(seq 0 6); do
    if timeout 3 ros2 topic echo "/drone_${i}_pcl_render_node/cloud" --once &>/dev/null; then
        echo -e "Drone $i: ${GREEN}[PASS]${NC} local cloud publishing"
        ((PASS++))
    else
        echo -e "Drone $i: ${RED}[FAIL]${NC} local cloud not publishing"
        ((FAIL++))
    fi
done
echo ""

# 3. Per-drone occupancy maps
echo "=== Per-Drone Occupancy Maps (SLAM) ==="
for i in $(seq 0 6); do
    if timeout 5 ros2 topic echo "/drone_${i}_ego_planner_node/grid_map/occupancy_inflate" --once &>/dev/null; then
        echo -e "Drone $i: ${GREEN}[PASS]${NC} occupancy_inflate publishing"
        ((PASS++))
    else
        echo -e "Drone $i: ${YELLOW}[WARN]${NC} occupancy_inflate not publishing yet"
        ((FAIL++))
    fi
done
echo ""

echo "============================================"
echo -e " Results: ${GREEN}$PASS PASS${NC}, ${RED}$FAIL FAIL${NC}"
echo "============================================"
echo ""
echo "In RViz2, you should see:"
echo "  - Gray obstacle point cloud (global map)"
echo "  - Colored occupancy maps around each drone"
echo "  - Obstacles highlighted as the drone approaches them"
