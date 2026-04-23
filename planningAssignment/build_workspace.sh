#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash

cd "$SCRIPT_DIR"
colcon build --packages-select rtabmap_diff_drive_tutorial bug_navigation

echo

echo "Build complete. Source with: source $SCRIPT_DIR/install/setup.bash"
