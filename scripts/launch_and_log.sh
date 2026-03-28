#!/bin/bash
# Launch a formation with full terminal logging
# Usage: ./scripts/launch_and_log.sh <formation> [goal_x] [goal_y] [goal_z]
# Examples:
#   ./scripts/launch_and_log.sh star
#   ./scripts/launch_and_log.sh arrow
#   ./scripts/launch_and_log.sh normal_hexagon

FORMATION=${1:?Usage: $0 <star|arrow|normal_hexagon> [goal_x] [goal_y] [goal_z]}
GOAL_X=${2:-25}
GOAL_Y=${3:-0}
GOAL_Z=${4:-1}

# Drone count per formation
case "$FORMATION" in
  star|arrow)       DRONE_COUNT=10 ;;
  normal_hexagon)   DRONE_COUNT=7 ;;
  sutd)             DRONE_COUNT=30 ;;
  *)                echo "Unknown formation: $FORMATION"; exit 1 ;;
esac

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$HOME/Swarm-Formation-ROS2/logs/${FORMATION}_${TIMESTAMP}"
mkdir -p "$LOG_DIR"

echo "=== Formation: $FORMATION | Drones: $DRONE_COUNT | Goal: ($GOAL_X, $GOAL_Y, $GOAL_Z) ==="
echo "=== Logs: $LOG_DIR ==="

# Source environment
source ~/Swarm-Formation-ROS2/activate.sh

# 1. Launch formation nodes
echo "[1/3] Launching ${FORMATION}_launch.py ..."
ros2 launch ego_planner ${FORMATION}_launch.py \
  2>&1 | tee "$LOG_DIR/launch.log" &
LAUNCH_PID=$!

# 2. Wait for all drones to reach WAIT_TARGET
echo "[2/3] Waiting for all $DRONE_COUNT drones to reach WAIT_TARGET ..."
READY=0
TIMEOUT=180
ELAPSED=0
while [ $READY -lt $DRONE_COUNT ] && [ $ELAPSED -lt $TIMEOUT ]; do
  sleep 2
  ELAPSED=$((ELAPSED + 2))
  READY=$(grep -c "from INIT to WAIT_TARGET" "$LOG_DIR/launch.log" 2>/dev/null || echo 0)
  echo "  ... $READY/$DRONE_COUNT ready (${ELAPSED}s)"
done

if [ $READY -lt $DRONE_COUNT ]; then
  echo "WARNING: Only $READY/$DRONE_COUNT drones ready after ${TIMEOUT}s timeout"
  echo "Check $LOG_DIR/launch.log for errors"
fi

# 3. Send goal
echo "[3/3] Sending goal ($GOAL_X, $GOAL_Y, $GOAL_Z) to $DRONE_COUNT drones ..."
./scripts/send_goal.sh "$GOAL_X" "$GOAL_Y" "$GOAL_Z" "$DRONE_COUNT" \
  2>&1 | tee "$LOG_DIR/goal.log"

echo ""
echo "=== Running. Press Ctrl+C to stop. ==="
echo "=== Logs are being written to: $LOG_DIR/launch.log ==="

# Wait for launch process (Ctrl+C will kill it)
wait $LAUNCH_PID 2>/dev/null

# Post-run: extract key diagnostics
echo ""
echo "=== Extracting diagnostics ==="

grep -E "\[SEQ_START\]|Failed to generate|SWARM_COLLISION|occ=1|from .* to" \
  "$LOG_DIR/launch.log" > "$LOG_DIR/diagnostics.log" 2>/dev/null

grep -E "iter=.*drone_id=" "$LOG_DIR/launch.log" > "$LOG_DIR/optimizer.log" 2>/dev/null

grep -E "FORMATION_DBG|formation_cost" "$LOG_DIR/launch.log" > "$LOG_DIR/formation.log" 2>/dev/null

echo "Diagnostics saved to:"
echo "  $LOG_DIR/diagnostics.log  (FSM transitions, stuck drones, collisions)"
echo "  $LOG_DIR/optimizer.log    (optimizer iterations per drone)"
echo "  $LOG_DIR/formation.log    (formation cost convergence)"
echo "  $LOG_DIR/launch.log       (full output)"
