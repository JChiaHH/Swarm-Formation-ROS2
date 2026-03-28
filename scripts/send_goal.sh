#!/bin/bash
# Send a goal pose to trigger all 7 drones to move in formation
# Usage: ./scripts/send_goal.sh [x] [y] [z] [drone_count]
# Default goal: (25.0, 0.0, 1.0)

X=${1:-25.0}
Y=${2:-0.0}
Z=${3:-1.0}

echo "Sending goal: ($X, $Y, $Z) on /goal_pose"

DRONE_COUNT=${4:-7}
echo "Waiting for $DRONE_COUNT subscribers..."

ros2 topic pub -t 20 -w "$DRONE_COUNT" /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, pose: {position: {x: $X, y: $Y, z: $Z}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

echo "Goal sent! Drones should start planning and moving."
