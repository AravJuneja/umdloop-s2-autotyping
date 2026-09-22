#!/usr/bin/env bash
# Time the typing stack end to end across N random launch codes.
#
# One simulator recreate per code, because the launch key is fixed when the
# sim container starts, and `dev` shares sim's network namespace so it has to
# come back with it. Each code gets one run, which covers one episode.
#
# Prints a per-episode table and summary statistics. Writes raw launch logs
# to log/e2e-stats/ so a run can be re-examined afterwards.
#
# Usage: ./scripts/e2e-stats.sh [count]     (default 5)
#        DUR=90 ./scripts/e2e-stats.sh 10   (longer per-code window)
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COUNT="${1:-5}"
DUR="${DUR:-75}"
PROJECT="${AUTOTYPE_PROJECT:-urcstats}"
SUFFIX="${AUTOTYPE_SUFFIX:--stats}"
PORT="${AUTOTYPE_PORT:-8097}"
DEV="urc-autotype-dev${SUFFIX}"
OUT="$REPO/log/e2e-stats"

mkdir -p "$OUT"; rm -f "$OUT"/*.log
codes="$(bash "$REPO/gen_codes.sh" "$COUNT")"
echo "codes: $codes"

cleanup() { (cd "$REPO/sim" && docker compose -p "$PROJECT" down >/dev/null 2>&1); }
trap cleanup EXIT

for code in $codes; do
    (cd "$REPO/sim" && AUTOTYPE_PORT="$PORT" AUTOTYPE_SUFFIX="$SUFFIX" \
        AUTOTYPE_SEED=1 AUTOTYPE_LAUNCH_KEY="$code" \
        docker compose -p "$PROJECT" up -d --force-recreate >/dev/null 2>&1)

    # Ready means frames are actually arriving, not merely that the topic is
    # listed: after a recreate, discovery sometimes never completes and the
    # stack would sit at frames=0 for the whole window.
    ready=no
    for _ in $(seq 1 25); do
        if docker exec "$DEV" bash -lc 'set +u
            source /opt/ros/jazzy/setup.bash; source /opt/autotype/install/setup.bash
            timeout 8 ros2 topic hz /camera/image_raw 2>/dev/null | grep -q "average rate"' 2>/dev/null
        then ready=yes; break; fi
        sleep 2
    done
    if [ "$ready" != yes ]; then
        echo "=== $code: SKIPPED, camera never streamed ==="
        continue
    fi

    # Build inside the container the same way `make e2e` does. The install
    # tree lands in the bind-mounted member_ws/, so this is only expensive
    # once; without it a fresh clone has no /ws/install to source.
    docker exec "$DEV" bash -lc 'set +u
        source /opt/ros/jazzy/setup.bash; source /opt/autotype/install/setup.bash
        cd /ws && colcon build --symlink-install' >/dev/null 2>&1 || {
        echo "=== $code: SKIPPED, colcon build failed ==="; continue; }

    echo "=== $code: streaming, running ${DUR}s ==="
    docker exec "$DEV" bash -lc "set +u
        source /opt/ros/jazzy/setup.bash
        source /opt/autotype/install/setup.bash
        source /ws/install/setup.bash
        timeout $DUR ros2 launch typist typist.launch.py > /tmp/code_$code.log 2>&1" \
        >/dev/null 2>&1
    docker cp "$DEV:/tmp/code_$code.log" "$OUT/$code.log" >/dev/null 2>&1
    echo "    $(grep -ac 'typed ' "$OUT/$code.log" 2>/dev/null || echo 0) episode(s) completed"
done

echo
python3 "$REPO/scripts/parse-stats.py" "$OUT"
