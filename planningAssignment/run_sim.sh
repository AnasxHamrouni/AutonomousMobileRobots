#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash

if [[ -f /usr/share/gazebo/setup.bash ]]; then
  source /usr/share/gazebo/setup.bash
elif [[ -f /usr/share/gazebo/setup.sh ]]; then
  source /usr/share/gazebo/setup.sh
fi

if [[ ! -f "$SCRIPT_DIR/install/setup.bash" ]]; then
	echo "Workspace not built yet. Run: $SCRIPT_DIR/build_workspace.sh"
	exit 1
fi

source "$SCRIPT_DIR/install/setup.bash"

exec ros2 launch rtabmap_diff_drive_tutorial robot_simulation.launch.py "$@"
