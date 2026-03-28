#!/bin/bash
# Source this file to activate the Swarm-Formation ROS2 environment
# Usage: source activate.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

eval "$(conda shell.bash hook)"
conda activate swarm_ros2

# Source ROS2 base setup
source "${CONDA_PREFIX}/setup.bash" 2>/dev/null || true

# Source the workspace overlay
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/install/setup.bash"
    echo "Swarm-Formation ROS2 workspace activated!"
else
    echo "WARNING: Workspace not built yet. Run ./setup_conda_env.sh first."
fi
