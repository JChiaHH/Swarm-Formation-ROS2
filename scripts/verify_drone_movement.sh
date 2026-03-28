#!/bin/bash
# Verify all 7 drones are moving after a goal is sent
# Usage: 1) Launch the system  2) Send a goal  3) Run this script

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

MOVING=0
STATIC=0

echo "============================================"
echo " Drone Movement Verification"
echo "============================================"
echo ""
echo "Taking 2 position snapshots 3 seconds apart..."
echo ""

declare -A X1_ARR Y1_ARR Z1_ARR

# First snapshot
for i in $(seq 0 6); do
    TOPIC="/drone_${i}_visual_slam/odom"
    POS=$(timeout 2 ros2 topic echo "$TOPIC" --once 2>/dev/null | grep -A3 "position:" | head -4)
    if [ -z "$POS" ]; then
        echo -e "Drone $i: ${RED}[NO ODOM]${NC}"
        X1_ARR[$i]="NONE"
        continue
    fi
    X1_ARR[$i]=$(echo "$POS" | grep "x:" | head -1 | awk '{print $2}')
    Y1_ARR[$i]=$(echo "$POS" | grep "y:" | head -1 | awk '{print $2}')
    Z1_ARR[$i]=$(echo "$POS" | grep "z:" | head -1 | awk '{print $2}')
    echo "Drone $i t=0: (${X1_ARR[$i]}, ${Y1_ARR[$i]}, ${Z1_ARR[$i]})"
done

echo ""
sleep 3
echo "Second snapshot:"
echo ""

# Second snapshot + compare
for i in $(seq 0 6); do
    if [ "${X1_ARR[$i]}" = "NONE" ]; then
        ((STATIC++))
        continue
    fi
    TOPIC="/drone_${i}_visual_slam/odom"
    POS=$(timeout 2 ros2 topic echo "$TOPIC" --once 2>/dev/null | grep -A3 "position:" | head -4)
    X2=$(echo "$POS" | grep "x:" | head -1 | awk '{print $2}')
    Y2=$(echo "$POS" | grep "y:" | head -1 | awk '{print $2}')
    Z2=$(echo "$POS" | grep "z:" | head -1 | awk '{print $2}')
    echo "Drone $i t=3: ($X2, $Y2, $Z2)"

    CHANGED=$(awk "BEGIN { dx=$X2-${X1_ARR[$i]}; dy=$Y2-${Y1_ARR[$i]}; dz=$Z2-${Z1_ARR[$i]}; d=sqrt(dx*dx+dy*dy+dz*dz); print (d > 0.01) ? 1 : 0 }" 2>/dev/null || echo "0")
    if [ "$CHANGED" = "1" ]; then
        echo -e "  -> ${GREEN}[MOVING]${NC}"
        ((MOVING++))
    else
        echo -e "  -> ${YELLOW}[STATIC]${NC} (send a goal first: ./scripts/send_goal.sh)"
        ((STATIC++))
    fi
done

echo ""
echo "============================================"
echo -e " ${GREEN}$MOVING moving${NC}, ${YELLOW}$STATIC static${NC} / 7 drones"
echo "============================================"
