#!/bin/bash
# Launch star formation with coredumps enabled to catch the drone crash
# Usage: ./scripts/launch_and_log_debug.sh

FORMATION=${1:-star}
GOAL_X=${2:-25}
GOAL_Y=${3:-0}
GOAL_Z=${4:-1}

case "$FORMATION" in
  star|arrow)       DRONE_COUNT=10 ;;
  normal_hexagon)   DRONE_COUNT=7 ;;
  *)                echo "Unknown formation: $FORMATION"; exit 1 ;;
esac

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$HOME/Swarm-Formation-ROS2/logs/${FORMATION}_${TIMESTAMP}"
mkdir -p "$LOG_DIR"

echo "=== Debug Launch: $FORMATION | Logs: $LOG_DIR ==="

# Enable coredumps
ulimit -c unlimited
export ASAN_OPTIONS=abort_on_error=1

source ~/Swarm-Formation-ROS2/activate.sh

# Launch with output captured
ros2 launch ego_planner ${FORMATION}_launch.py \
  2>&1 | tee "$LOG_DIR/launch.log" &
LAUNCH_PID=$!

# Wait for drones
echo "Waiting for $DRONE_COUNT drones..."
READY=0
TIMEOUT=180
ELAPSED=0
while [ $READY -lt $DRONE_COUNT ] && [ $ELAPSED -lt $TIMEOUT ]; do
  sleep 2
  ELAPSED=$((ELAPSED + 2))
  READY=$(grep -c "from INIT to WAIT_TARGET" "$LOG_DIR/launch.log" 2>/dev/null || echo 0)
  echo "  ... $READY/$DRONE_COUNT ready (${ELAPSED}s)"
done

# Send goal
echo "Sending goal..."
./scripts/send_goal.sh "$GOAL_X" "$GOAL_Y" "$GOAL_Z" "$DRONE_COUNT" \
  2>&1 | tee "$LOG_DIR/goal.log"

echo ""
echo "=== Running. Press Ctrl+C to stop. ==="
echo "=== After crash, check for core dumps: find /tmp -name 'core*' -newer $LOG_DIR/launch.log ==="

wait $LAUNCH_PID 2>/dev/null

# Extract diagnostics
echo "=== Extracting diagnostics ==="
grep -E "\[SEQ_START\]|Failed to generate|SWARM_COLLISION|occ=1|process has died|from .* to" \
  "$LOG_DIR/launch.log" > "$LOG_DIR/diagnostics.log" 2>/dev/null
grep -E "iter=.*drone_id=" "$LOG_DIR/launch.log" > "$LOG_DIR/optimizer.log" 2>/dev/null

echo "Done. Key files:"
echo "  $LOG_DIR/diagnostics.log"
echo "  $LOG_DIR/optimizer.log"
echo "  $LOG_DIR/launch.log"

# Check for core dumps
CORES=$(find /tmp /var/crash "$HOME" -maxdepth 2 -name 'core*' -newer "$LOG_DIR/launch.log" 2>/dev/null)
if [ -n "$CORES" ]; then
  echo ""
  echo "=== Core dumps found: ==="
  echo "$CORES"
  echo "Run: gdb /path/to/ego_planner_node <core_file> -ex bt -ex quit"
fi
