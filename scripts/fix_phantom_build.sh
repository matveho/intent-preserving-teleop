#!/usr/bin/env bash
set -euo pipefail

# Run from any directory; supply the ROS workspace used for the SAW import.
workspace="${1:-$HOME/ros_ws}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
patch_file="$script_dir/patches/sawSensablePhantom-cisst-1.5.patch"

if [[ ! -d "$workspace/src" ]]; then
    echo "Workspace source directory does not exist: $workspace/src" >&2
    exit 1
fi
mapfile -d '' candidates < <(find "$workspace/src" -type d -name sawSensablePhantom -prune -print0)
source_dirs=()
for candidate in "${candidates[@]}"; do
    [[ -f "$candidate/core/CMakeLists.txt" && -f "$candidate/ros/CMakeLists.txt" ]] && source_dirs+=("$candidate")
done
if [[ ${#source_dirs[@]} -ne 1 ]]; then
    echo "Expected one sawSensablePhantom source checkout under $workspace/src; found ${#source_dirs[@]}." >&2
    exit 1
fi
phantom_source="${source_dirs[0]}"
if git -C "$phantom_source" apply --reverse --check "$patch_file" 2>/dev/null; then
    echo "Compatibility patch already applied."
elif git -C "$phantom_source" apply --check "$patch_file"; then
    git -C "$phantom_source" apply "$patch_file"
    echo "Applied compatibility patch to $phantom_source"
else
    echo "Source differs from the tested revision; no patch applied. Inspect local edits before proceeding." >&2
    exit 1
fi
# Ignore obsolete ROS 1 packages during colcon discovery and rosdep scanning.
touch "$phantom_source/core/share/COLCON_IGNORE" "$phantom_source/igtl/COLCON_IGNORE"
echo "Ready to rebuild. This script does not install packages or run the device."
