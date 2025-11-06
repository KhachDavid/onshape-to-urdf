#!/usr/bin/env bash
set -euo pipefail

# Demo script to visualize the latest generated URDF in RViz2.
# Usage:
#   bash rviz_demo.sh [OUTPUT_DIR]
# If OUTPUT_DIR is not provided, the newest directory under output/ is used.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
REPO_ROOT="$SCRIPT_DIR"

OUTPUT_DIR_INPUT="${1:-}"

# Track background process PIDs for cleanup
declare -a BG_PIDS=()

err() { echo "[rviz_demo] $*" >&2; }
die() { err "$*"; exit 1; }

# Cleanup function to kill all background processes
cleanup() {
  echo
  echo "[rviz_demo] Shutting down gracefully..."
  
  # Kill all tracked background processes
  for pid in "${BG_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      echo "[rviz_demo] Stopping process $pid"
      kill "$pid" 2>/dev/null || true
    fi
  done
  
  # Wait a moment for graceful shutdown
  sleep 0.5
  
  # Force kill any remaining processes
  for pid in "${BG_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      echo "[rviz_demo] Force stopping process $pid"
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
  
  echo "[rviz_demo] Cleanup complete."
}

# Set up trap to catch EXIT, SIGINT (Ctrl+C), and SIGTERM
trap cleanup EXIT INT TERM

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Required command '$1' not found in PATH"
}

# Ensure ROS 2 tools are available
require_cmd ros2
require_cmd rviz2

# Resolve output directory
if [[ -n "$OUTPUT_DIR_INPUT" ]]; then
  OUTPUT_DIR="$(cd "$OUTPUT_DIR_INPUT" 2>/dev/null && pwd)" || die "Invalid output directory: $OUTPUT_DIR_INPUT"
else
  if [[ ! -d "$REPO_ROOT/output" ]]; then
    die "No output/ directory found. Run the conversion first (e.g., bash run_conversion.sh)."
  fi
  # Pick the most recent timestamped output directory
  LATEST_DIR="$(ls -1dt "$REPO_ROOT"/output/*/ 2>/dev/null | head -n 1 || true)"
  [[ -n "$LATEST_DIR" ]] || die "No output subdirectories found. Run the conversion first."
  OUTPUT_DIR="${LATEST_DIR%/}"
fi

URDF_PATH=""
if [[ -f "$OUTPUT_DIR/robot_standalone.urdf" ]]; then
  URDF_PATH="$OUTPUT_DIR/robot_standalone.urdf"
elif [[ -f "$OUTPUT_DIR/robot.urdf" ]]; then
  URDF_PATH="$OUTPUT_DIR/robot.urdf"
else
  die "Could not find URDF in '$OUTPUT_DIR' (expected robot_standalone.urdf or robot.urdf)"
fi

echo "[rviz_demo] Using output directory: $OUTPUT_DIR"
echo "[rviz_demo] Using URDF: $URDF_PATH"

# Launch a joint state publisher (GUI if available, else headless)
JSP_CMD=()
if ros2 pkg prefix joint_state_publisher_gui >/dev/null 2>&1; then
  JSP_CMD=(ros2 run joint_state_publisher_gui joint_state_publisher_gui)
elif ros2 pkg prefix joint_state_publisher >/dev/null 2>&1; then
  JSP_CMD=(ros2 run joint_state_publisher joint_state_publisher)
else
  err "Neither joint_state_publisher_gui nor joint_state_publisher package found. Skipping joint publisher."
fi

if [[ ${#JSP_CMD[@]} -gt 0 ]]; then
  echo "[rviz_demo] Starting joint state publisher in background..."
  "${JSP_CMD[@]}" >/dev/null 2>&1 &
  BG_PIDS+=($!)
fi

echo "[rviz_demo] Starting robot_state_publisher in background..."
# Convert relative mesh paths to absolute for robot_state_publisher
URDF_CONTENT="$(cat "$URDF_PATH" | sed "s|filename=\"assets/|filename=\"file://$OUTPUT_DIR/assets/|g" | sed "s|filename=\"meshes/|filename=\"file://$OUTPUT_DIR/meshes/|g")"
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$URDF_CONTENT" \
  -p publish_frequency:=30.0 >/dev/null 2>&1 &
BG_PIDS+=($!)

# Give robot_state_publisher a moment to start
sleep 1

# Use bundled RViz config if present
RVIZ_CONFIG="$REPO_ROOT/rviz/demo.rviz"
if [[ -f "$RVIZ_CONFIG" ]]; then
  echo "[rviz_demo] Launching RViz2 with config: $RVIZ_CONFIG"
  rviz2 -d "$RVIZ_CONFIG"
else
  echo "[rviz_demo] Launching RViz2 (no config found)."
  rviz2
fi

# When RViz exits, the cleanup trap will automatically run


