#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DB_PATH="${1:-$SCRIPT_DIR/maps/current_world.db}"

source /opt/ros/humble/setup.bash
if [[ -f /usr/share/gazebo/setup.bash ]]; then
  source /usr/share/gazebo/setup.bash
elif [[ -f /usr/share/gazebo/setup.sh ]]; then
  source /usr/share/gazebo/setup.sh
fi

# More robust rendering on systems where RTAB-Map/RViz panels appear black.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

if [[ ! -f "$SCRIPT_DIR/install/setup.bash" ]]; then
  echo "Workspace not built yet. Run: $SCRIPT_DIR/build_workspace.sh"
  exit 1
fi

mkdir -p "$SCRIPT_DIR/maps"
source "$SCRIPT_DIR/install/setup.bash"

exec ros2 launch rtabmap_diff_drive_tutorial rtabmap_slam.launch.py \
  database_path:="$DB_PATH" \
  use_rtabmap_viz:=false
