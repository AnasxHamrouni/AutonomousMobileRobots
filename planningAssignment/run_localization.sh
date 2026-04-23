#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DB_PATH="${1:-$SCRIPT_DIR/maps/current_world.db}"
INITIAL_POSE="${2:-0 0 0 0 0 0}"

source /opt/ros/humble/setup.bash

if [[ ! -f "$SCRIPT_DIR/install/setup.bash" ]]; then
	echo "Workspace not built yet. Run: $SCRIPT_DIR/build_workspace.sh"
	exit 1
fi

if [[ ! -f "$DB_PATH" ]]; then
	echo "Map database not found at: $DB_PATH"
	exit 1
fi

source "$SCRIPT_DIR/install/setup.bash"

exec ros2 launch rtabmap_diff_drive_tutorial rtabmap_no_mapping.launch.py use_sim_time:=true database_path:="$DB_PATH" initial_pose:="$INITIAL_POSE"
