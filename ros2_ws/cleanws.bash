# ROS 2 workspace cleanup helper
ros2_purge() {
    if [ -z "$1" ]; then
        echo "Usage: ros2_purge [workspace_root]"
        echo "  If no path provided, uses current directory."
        return 1
    fi

    local WS_ROOT="${1:-$PWD}"

    # Safety check: only run if the proper structure with 'build', 'install', 'src', and 'log' exist:
    if [[ ! -d "$WS_ROOT/src" ]]; then
        echo "Error: '$WS_ROOT' does not look like a ROS 2 workspace (no 'src' directory)."
        return 1
    fi

    echo "Purging workspace: $WS_ROOT"
    echo "  → Removing: build install log"
    
    # Remove the generated directories
    rm -rf "$WS_ROOT/build" "$WS_ROOT/install" "$WS_ROOT/log" 2>/dev/null || true

    # Optional: remove any lingering .cmake files in src (rare but can happen)
    find "$WS_ROOT/src" -name "CMakeCache.txt" -delete
    find "$WS_ROOT/src" -name "CMakeFiles" -type d -exec rm -rf {} + 2>/dev/null || true

    echo "Workspace purged. Ready for clean rebuild."
    echo "Next: colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
}