#!/usr/bin/env bash
# Staged build: everything P4 needs first, then the rest of the workspace.
#
# The shared heavy lock is scarce (the other research line often holds it for ~40 min
# stretches), so the two packages that make the visual edges testable are built first
# and their result is reported immediately; the full workspace build follows in the
# same lock window.
set +u
LINE_ROOT=/home/wcqw/explorb_line_20261121
LOG=$LINE_ROOT/logs/build_final.log

{
  echo "=== stage 1: visual_query + teb_local_planner ($(date +%H:%M:%S)) ==="
  "$LINE_ROOT/build.sh" -j 8 --pkg visual_query --pkg teb_local_planner
  RC1=$?
  echo "=== stage 1 rc=$RC1 ==="
  ls -l "$LINE_ROOT/ws/devel/lib/visual_query/" 2>/dev/null
  ls -l "$LINE_ROOT/ws/devel/lib/libteb_local_planner.so" 2>/dev/null

  echo "=== stage 2: full workspace (orb_slam2_ros mask plumbing) ($(date +%H:%M:%S)) ==="
  "$LINE_ROOT/build.sh" -j 8
  RC2=$?
  echo "=== stage 2 rc=$RC2 ==="
  echo "SUMMARY stage1=$RC1 stage2=$RC2"
} 2>&1 | tee "$LOG"
