#!/bin/bash
# =============================================================================
# Swarm-Formation ROS2 Jazzy - Conda Environment Setup
# =============================================================================
# This script creates an isolated conda environment with ROS2 Jazzy and all
# dependencies needed to build and run the Swarm-Formation project.
# =============================================================================

set -e

ENV_NAME="swarm_ros2"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================"
echo " Swarm-Formation ROS2 Jazzy Setup"
echo "============================================"

# ---- Step 1: Create conda environment with ROS2 Jazzy ----
echo ""
echo "[1/5] Creating conda environment '${ENV_NAME}' with ROS2 Jazzy..."

# Check if environment already exists
if conda env list | grep -q "^${ENV_NAME} "; then
    echo "  Environment '${ENV_NAME}' already exists. Removing and recreating..."
    conda env remove -n ${ENV_NAME} -y
fi

# Create environment with robostack ROS2 Jazzy
# Using conda-forge and robostack channels for ROS2
conda create -n ${ENV_NAME} -y python=3.11

echo "[1/5] Done."

# ---- Step 2: Install ROS2 Jazzy packages via robostack ----
echo ""
echo "[2/5] Installing ROS2 Jazzy and dependencies via robostack..."

# Activate environment for package installation
eval "$(conda shell.bash hook)"
conda activate ${ENV_NAME}

# Add robostack channel for ROS2 Jazzy
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --set channel_priority strict

# Install ROS2 Jazzy base + required packages
conda install -y \
    ros-jazzy-desktop \
    ros-jazzy-ament-cmake \
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    ros-jazzy-std-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-message-filters \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-common \
    ros-jazzy-rviz-default-plugins \
    ros-jazzy-rviz-rendering \
    ros-jazzy-rclcpp-components \
    ros-jazzy-launch-ros \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    ros-jazzy-pluginlib \
    ros-jazzy-builtin-interfaces \
    colcon-common-extensions \
    compilers \
    cmake \
    pkg-config

echo "[2/5] Done."

# ---- Step 3: Install additional system dependencies ----
echo ""
echo "[3/5] Installing additional dependencies (Eigen3, PCL, OpenCV, Armadillo)..."

conda install -y \
    eigen \
    pcl \
    opencv \
    armadillo \
    nlohmann_json \
    libboost-devel

# Install tf_transformations for Python scripts
pip install tf_transformations transforms3d

echo "[3/5] Done."

# ---- Step 4: Build the workspace ----
echo ""
echo "[4/5] Building the ROS2 workspace..."

cd "${SCRIPT_DIR}"

# Source ROS2 setup
source "${CONDA_PREFIX}/setup.bash" 2>/dev/null || true

# Build with colcon
colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
    --parallel-workers $(nproc) \
    2>&1 | tee build_log.txt

BUILD_STATUS=${PIPESTATUS[0]}

echo "[4/5] Done."

# ---- Step 5: Post-build setup ----
echo ""
echo "[5/5] Setting up environment activation script..."

# Create a convenience activation script
cat > "${SCRIPT_DIR}/activate.sh" << 'ACTIVATE_EOF'
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
ACTIVATE_EOF

chmod +x "${SCRIPT_DIR}/activate.sh"

echo "[5/5] Done."

# ---- Summary ----
echo ""
echo "============================================"
echo " Setup Complete!"
echo "============================================"

if [ ${BUILD_STATUS} -eq 0 ]; then
    echo " Build: SUCCESS"
else
    echo " Build: FAILED (check build_log.txt for errors)"
    echo ""
    echo " To retry the build manually:"
    echo "   source activate.sh"
    echo "   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
fi

echo ""
echo " To activate the environment:"
echo "   source activate.sh"
echo ""
echo " To run the simulation (7-drone hexagonal formation):"
echo "   ros2 launch plan_manage normal_hexagon_launch.py"
echo ""
echo " To visualize in RViz2:"
echo "   ros2 launch plan_manage rviz_launch.py"
echo ""
echo "============================================"
