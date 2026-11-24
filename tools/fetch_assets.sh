#!/usr/bin/env bash
# Re-download the two large assets that are not kept in git:
#   1. the Gazebo model set of the AWS bookstore world (~190 MB)
#   2. the ORB vocabulary in binary form (48 MB)
# Both come from the public ExplORB-SLAM repository (author's files, unchanged).
set +u
LINE=${LINE_ROOT:-/home/wcqw/explorb_line_20261121}
BASE=https://raw.githubusercontent.com/JulioPlaced/ExplORB-SLAM/main

echo "[1/2] ORB vocabulary (binary, 48 MB)"
mkdir -p "$LINE/ws/src/orb_slam2_ros/orb_slam2/Vocabulary"
V="$LINE/ws/src/orb_slam2_ros/orb_slam2/Vocabulary/ORBvoc.txt.bin"
for i in 1 2 3 4 5; do
    curl -sL -C - --retry 5 -o "$V" \
      "$BASE/src/3dParty/orb_slam_2_ros/orb_slam2/Vocabulary/ORBvoc.txt.bin"
    SZ=$(stat -c%s "$V" 2>/dev/null || echo 0)
    echo "  attempt $i: $SZ bytes"
    [ "$SZ" -ge 48000000 ] && break
done

echo "[2/2] AWS Gazebo models (~190 MB)"
python3 - "$LINE" <<'PY'
import json, os, sys, time, urllib.request
import concurrent.futures as cf
LINE = sys.argv[1]
BASE = "https://raw.githubusercontent.com/JulioPlaced/ExplORB-SLAM/main/"
ROOT = os.path.join(LINE, "ws/src/robot_description/launch/include")
tree = json.load(urllib.request.urlopen(
    "https://api.github.com/repos/JulioPlaced/ExplORB-SLAM/git/trees/main?recursive=1"))
rows = [(x['path'], x.get('size', 0)) for x in tree['tree'] if x['type'] == 'blob'
        and x['path'].startswith('src/robot_description/launch/include/models/')]

def get(row):
    p, s = row
    dst = os.path.join(ROOT, p[len('src/robot_description/launch/include/'):])
    if os.path.exists(dst) and os.path.getsize(dst) == s:
        return 0
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    for a in range(3):
        try:
            with urllib.request.urlopen(BASE + p, timeout=90) as r, open(dst, 'wb') as f:
                f.write(r.read())
            return 0
        except Exception:
            time.sleep(1)
    return 1

fails = 0
with cf.ThreadPoolExecutor(max_workers=16) as ex:
    for rc in ex.map(get, rows):
        fails += rc
print("  files=%d fails=%d" % (len(rows), fails))
PY
echo "done."
