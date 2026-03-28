#!/bin/bash
# Change formation type and optionally send a new goal
# Usage: ./scripts/change_formation.sh [formation_type] [goal_x] [goal_y] [goal_z]
#   2 = STAR_FORMATION
#   3 = ARROW_FORMATION

FORMATION_TYPE=${1:-3}
GOAL_X=${2:-}
GOAL_Y=${3:-0.0}
GOAL_Z=${4:-1.0}

echo "Changing formation to type $FORMATION_TYPE"

ros2 topic pub -t 5 -w 10 /change_formation std_msgs/msg/Int32 \
  "{data: $FORMATION_TYPE}"

echo "Formation change command sent!"

# If a new goal was provided, send it after a short delay
if [ -n "$GOAL_X" ]; then
  sleep 1
  echo "Sending new goal: ($GOAL_X, $GOAL_Y, $GOAL_Z)"
  ros2 topic pub -t 5 -w 10 /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
  echo "New goal sent!"
fi
