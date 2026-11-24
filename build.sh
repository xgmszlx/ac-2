#!/usr/bin/env bash
# Reproducible build of this line's catkin workspace.
#
#   sudo build.sh [-j N] [--pkg name ...]
#
# Two traps this script exists to avoid (both cost a whole round on 2026-11-25):
#
#  1. `source /opt/ros/noetic/setup.bash` and `source ws/devel/setup.bash` *replace*
#     ROS_PACKAGE_PATH with "<ws>/src:/opt/ros/noetic/share", which silently hides
#     every package that only lives in the no-sudo local prefix
#     (teb_local_planner, costmap_converter, octomap_ros, mbf_*). Symptom: move_base
#     dies with "Failed to create the teb_local_planner/TebLocalPlannerROS planner".
#  2. They do the same to CMAKE_PREFIX_PATH, which makes the *build* fail with
#     "Could not find 'share/costmap_converter/cmake/costmap_converter-msg-paths.cmake'".
#
# So both variables are re-prepended *after* the setup files, not before.
set +u

LINE_ROOT=/home/wcqw/explorb_line_20261121
LR=$LINE_ROOT/localroot

source "$LINE_ROOT/env.sh"
source "$LINE_ROOT/ws/devel/setup.bash" 2>/dev/null || true

export ROS_PACKAGE_PATH="$LR/opt/ros/noetic/share:$ROS_PACKAGE_PATH"
export CMAKE_PREFIX_PATH="$LR/opt/ros/noetic:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$LR/opt/ros/noetic/lib:$LR/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

JOBS=8
PKGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        -j) JOBS="$2"; shift 2 ;;
        -j*) JOBS="${1#-j}"; shift ;;
        --pkg) PKGS+=("$2"); shift 2 ;;
        *) echo "usage: build.sh [-j N] [--pkg name ...]"; exit 2 ;;
    esac
done

echo "[build] ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
echo "[build] CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
echo "[build] teb_local_planner -> $(rospack find teb_local_planner 2>/dev/null || echo MISSING)"
echo "[build] costmap_converter  -> $(rospack find costmap_converter 2>/dev/null || echo MISSING)"

# genmsg/`generate_messages` does NOT search CMAKE_PREFIX_PATH for its dependency
# msg-paths files: it searches CATKIN_WORKSPACES, which catkin derives from the
# CMAKE_PREFIX_PATH entries that *contain a .catkin marker* (catkin/cmake/all.cmake).
# The no-sudo local prefix has no such marker (and adding one is not enough on this
# machine's cmake/catkin combination), so the needed share/ trees are exposed inside
# the devel space, which is always a workspace. Without this, building a package that
# depends on costmap_converter/mbf msgs dies with
#   Could not find 'share/costmap_converter/cmake/costmap_converter-msg-paths.cmake'
mkdir -p "$LINE_ROOT/ws/devel/share"
for d in "$LR"/opt/ros/noetic/share/*; do
    [ -d "$d" ] || continue
    p=$(basename "$d")
    [ -e "$LINE_ROOT/ws/devel/share/$p" ] || ln -s "$d" "$LINE_ROOT/ws/devel/share/$p"
done
# the marker itself is harmless and makes the prefix a catkin workspace as well
touch "$LR/opt/ros/noetic/.catkin"

cd "$LINE_ROOT/ws" || exit 1
# catkin_make *always* passes its own -DCMAKE_PREFIX_PATH (=<ws>/devel:<whatever the
# catkin environment had>), so neither an exported CMAKE_PREFIX_PATH nor an extra -D
# survives: the first drops out and the second gets overridden. What does survive is a
# per-package <pkg>_DIR cache hint, so every package that only exists in the no-sudo
# local prefix is pointed at explicitly. Without this, the build of teb_local_planner
# stops at "Could not find share/costmap_converter/cmake/costmap_converter-msg-paths.cmake".
CMAKE_ARGS=(-DPYTHON_EXECUTABLE=/usr/bin/python3)
for d in "$LR"/opt/ros/noetic/share/*/cmake; do
    [ -d "$d" ] || continue
    pkg=$(basename "$(dirname "$d")")
    cfg=$(ls "$d"/*Config.cmake 2>/dev/null | head -1)
    [ -n "$cfg" ] || continue
    # never point a package at the local prefix copy of *itself* (teb_local_planner is
    # built from ws/src here, and a stale deb config would confuse its own build)
    [ -d "$LINE_ROOT/ws/src/$pkg" ] && continue
    CMAKE_ARGS+=("-D${pkg}_DIR=$d")
    echo "[build] hint ${pkg}_DIR=$d"
done

if [ ${#PKGS[@]} -eq 0 ]; then
    catkin_make "${CMAKE_ARGS[@]}" -j"$JOBS"
else
    for p in "${PKGS[@]}"; do
        echo "[build] === $p ==="
        catkin_make "${CMAKE_ARGS[@]}" -j"$JOBS" --pkg "$p" || exit $?
    done
fi
RC=$?
echo "[build] rc=$RC"
exit $RC
