#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENARIO_NAME="${SCENARIO_NAME:-default}"
METRICS_DIR="${METRICS_DIR:-$SCRIPT_DIR/metrics}"

source /opt/ros/humble/setup.bash

if [[ ! -f "$SCRIPT_DIR/install/setup.bash" ]]; then
	echo "Workspace not built yet. Run: $SCRIPT_DIR/build_workspace.sh"
	exit 1
fi

source "$SCRIPT_DIR/install/setup.bash"

echo "Starting Bug 1 Planner..."
if ros2 run bug_navigation bug1_planner_node --ros-args -p use_sim_time:=true -p scenario_name:="$SCENARIO_NAME" -p metrics_output_dir:="$METRICS_DIR" "$@"; then
	exit 0
fi

echo "ros2 run fallback: starting direct executable"
if [[ -x "$SCRIPT_DIR/install/bug_navigation/lib/bug_navigation/bug1_planner_node" ]]; then
	exec "$SCRIPT_DIR/install/bug_navigation/lib/bug_navigation/bug1_planner_node" --ros-args -p use_sim_time:=true -p scenario_name:="$SCENARIO_NAME" -p metrics_output_dir:="$METRICS_DIR" "$@"
fi

exec "$SCRIPT_DIR/install/bug_navigation/bin/bug1_planner_node" --ros-args -p use_sim_time:=true -p scenario_name:="$SCENARIO_NAME" -p metrics_output_dir:="$METRICS_DIR" "$@"
