#!/usr/bin/env bash
# Run the typing stack through N episodes, one per /sim/reset.
#
# Assumes the stack is already up: ./start.sh, then
#   ros2 launch typist typist.launch.py
# inside the dev container. Each iteration resets the sim (next seed),
# waits for the typist to finish, and checks /sim/result for an exact match.
# Stops at the first failure.
#
# Usage: SEEDS=3 ./e2e.sh   (default 3)
set -euo pipefail

SEEDS="${SEEDS:-3}"
TIMEOUT="${E2E_TIMEOUT:-90}"

SETUP_OK=0
# A "SETUP='a && b'" string followed by `$SETUP && cmd` does NOT compose:
# expansion never creates operators, so '&&' lands as arguments to `source`.
# A function does. `set +u` around the sources: the ROS setup scripts
# reference AMENT_TRACE_SETUP_FILES without a default under `set -u`.
setup_ros() {
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    # shellcheck disable=SC1091
    source /opt/autotype/install/setup.bash
    # shellcheck disable=SC1091
    source /ws/install/setup.bash
    set -u
}
# Measured 2026-09-21 on edc6965, 14 random codes of 3-6 characters, one
# episode each: 14/14 typed exactly. End to end 42.1 s mean (sd 4.6), of
# which cold start to first press is 19.9 s (sd 5.0) and typing is 22.1 s.
# Typing cost is linear in length at 4.78 s/char with no fixed overhead;
# 1.50 s of that per character is the two unconditional sleeps in
# typist._run. Reproduce with `make stats N=14`.
#
# Known gap: the typist types one episode and then idles rather than picking
# up the next, so the numbers above are all cold starts. Re-arming between
# episodes is untested.

for ((i = 1; i <= SEEDS; i++)); do
    echo "=== e2e seed $i/$SEEDS ==="
    # Start the checker FIRST so it observes the reset-clear + fresh
    # result ordering directly (no stale-latch race), then reset.
    setup_ros
    E2E_TIMEOUT="$TIMEOUT" ros2 run typist e2e_check --expect-new &
    checker=$!
    sleep 5 # let e2e_check subscribe before the reset clears the latch
    setup_ros && ros2 service call /sim/reset std_srvs/srv/Trigger '{}' >/dev/null
    if wait "$checker"; then
        echo "--- seed $i PASS ---"
    else
        echo "--- seed $i FAIL ---"
        exit 1
    fi
done
echo "ALL $SEEDS SEEDS PASS"
