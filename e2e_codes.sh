#!/usr/bin/env bash
# Type N fresh random launch codes, one simulator restart per code.
#
# Split-brain by necessity: the dev container has ROS but no docker CLI, while
# the host has compose but no ROS. The Makefile drives this loop on the host:
# generate codes, restart the sim container per code, and run one e2e_check in
# dev per code. Member nodes stay up in dev throughout.
#
# Usage: CODES=3 ./e2e_codes.sh   (default 3)
set -euo pipefail

CODES="${CODES:-3}"
TIMEOUT="${E2E_TIMEOUT:-90}"
MODE="${1:-host}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"

if [[ "$MODE" == "dev" ]]; then
    # Runs inside dev: start one bag + the typing stack, then one checker per
    # code. The host loop below restarts sim between checkers.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    # shellcheck disable=SC1091
    source /ws/install/setup.bash 2>/dev/null || true
    # shellcheck disable=SC1091
    source /opt/autotype/install/setup.bash
    # shellcheck disable=SC1091
    source /ws/install/setup.bash
    set -u
    LOG_DIR="/ws/log/e2e/${RUN_ID}"
    mkdir -p "$LOG_DIR"
    echo "$RUN_ID" > /tmp/e2e_run_id.txt
    codes="$(CODES="$CODES" bash /tmp/gen_codes.sh)"
    echo "$codes" > /tmp/e2e_codes.txt
    echo "codes: $codes"
    echo "logs: $LOG_DIR"
    nohup ros2 bag record -o "$LOG_DIR/bag" \
        /panel_detect/detections \
        /key_projector/panel_pose \
        /key_projector/key_positions \
        /joint_states \
        /arm/cmd_joint_velocity \
        /arm/press \
        /typist/press_decisions \
        /sim/launch_key \
        /sim/result >/tmp/bag.log 2>&1 &
    bag_pid=$!
    echo "$bag_pid" > /tmp/bag.pid
    (nohup ros2 launch typist typist.launch.py > /tmp/typist.log 2>&1 &)
    sleep 30
    for code in $codes; do
        echo "=== e2e code $code (waiting for sim) ==="
        while [[ ! -f /tmp/sim_ready.txt ]]; do sleep 1; done
        rm -f /tmp/sim_ready.txt
        if E2E_TIMEOUT="$TIMEOUT" ros2 run typist e2e_check \
            --code "$code" --csv "$LOG_DIR/results.csv"; then
            echo "--- code $code PASS ---"
            echo "PASS $code" >> /tmp/e2e_results.txt
        else
            echo "--- code $code FAIL ---"
            echo "FAIL $code" >> /tmp/e2e_results.txt
        fi
    done
    kill "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
    exit 0
fi

# Host mode: generate codes, restart sim per code, report.
codes="$(CODES="$CODES" bash gen_codes.sh)"
echo "codes: $codes"
fail=0
for code in $codes; do
    seed="$RANDOM"
    echo "=== e2e code $code seed $seed ==="
    (cd sim && AUTOTYPE_SEED="$seed" AUTOTYPE_LAUNCH_KEY="$code" \
        docker compose up -d --force-recreate sim >/tmp/sim_up.log 2>&1)
    sleep 15
    touch /tmp/sim_ready_host.txt
    echo "restarted sim for $code; run the dev checker now"
done
echo "host loop done (dev checker runs separately)"
