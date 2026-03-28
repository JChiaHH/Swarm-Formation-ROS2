#!/bin/bash
# Verify all expected ROS2 topics for the 7-drone swarm formation
# Usage: source activate.sh && ros2 launch plan_manage normal_hexagon_launch.py
#        Then in another terminal: source activate.sh && ./scripts/verify_topics.sh

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0

check_topic() {
    local topic="$1"
    local description="$2"
    if echo "$TOPIC_LIST" | grep -qx "$topic"; then
        echo -e "  ${GREEN}[PASS]${NC} $topic ($description)"
        ((PASS++))
    else
        echo -e "  ${RED}[FAIL]${NC} $topic ($description)"
        ((FAIL++))
    fi
}

echo "============================================"
echo " Swarm-Formation ROS2 Topic Verification"
echo "============================================"
echo ""

# Check if ROS2 is accessible
TOPIC_LIST=$(ros2 topic list 2>/dev/null)
if [ -z "$TOPIC_LIST" ]; then
    echo -e "${RED}ERROR: Cannot reach ROS2. Is the environment sourced?${NC}"
    exit 1
fi

TOPIC_COUNT=$(echo "$TOPIC_LIST" | wc -l)
echo "Found $TOPIC_COUNT active topics"
echo ""

# Global topics
echo "=== Global Topics ==="
check_topic "/map_generator/global_cloud" "obstacle map point cloud"
check_topic "/broadcast_traj_from_planner" "swarm traj broadcast send"
check_topic "/broadcast_traj_to_planner" "swarm traj broadcast recv"
echo ""

# Per-drone topics
for i in $(seq 0 6); do
    echo "=== Drone $i ==="
    check_topic "/drone_${i}_visual_slam/odom" "odometry"
    check_topic "/drone_${i}_planning/pos_cmd" "position command"
    check_topic "/drone_${i}_planning/trajectory" "polynomial trajectory"
    check_topic "/drone_${i}_pcl_render_node/cloud" "local point cloud"
    check_topic "/drone_${i}_ego_planner_node/grid_map/occupancy_inflate" "inflated occupancy"
    check_topic "/drone_${i}_ego_planner_node/optimal_list" "optimal trajectory viz"
    check_topic "/drone_${i}_odom_visualization/robot" "robot marker"
    check_topic "/drone_${i}_odom_visualization/path" "drone path"
    echo ""
done

echo "=== Swarm Visualization ==="
check_topic "/drone_0_ego_planner_node/swarm_graph_visual" "formation graph"
echo ""

echo "============================================"
echo -e " Results: ${GREEN}$PASS PASS${NC}, ${RED}$FAIL FAIL${NC}"
echo "============================================"
