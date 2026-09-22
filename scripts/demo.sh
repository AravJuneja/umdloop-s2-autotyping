#!/usr/bin/env bash
# Watchable demo: type several launch codes, one episode per seed/key pair,
# with the dashboard live on http://localhost:8080 the whole time.
#
# Uses the default compose project and port, so the dashboard URL never
# changes even though the simulator is recreated per run. `dev` shares sim's
# network namespace, so both containers come back together each time.
#
# Per run it records the evidence topics (detections, pose estimates,
# commands, press decisions, results) with `ros2 bag record`, converts the bag
# to a human-readable JSONL log, and copies both -- plus the launch log -- to
# log/demo/<code>/. That survives the per-run container recreate, which would
# otherwise discard everything left in /tmp.
#
# The stack is torn down when the run finishes (or is interrupted).
#
# Usage: ./scripts/demo.sh [runs]      (default 3)
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNS="${1:-3}"
DUR="${DUR:-75}"
DEV=urc-autotype-dev

cleanup() { (cd "$REPO/sim" && docker compose down >/dev/null 2>&1); }
trap cleanup EXIT

# Where the evidence lands, and what goes into the bag. Keep this on one
# line: it is expanded into a `bash -lc` string, where a newline would be
# re-parsed as a command separator rather than a topic separator.
OUT="$REPO/log/demo"
BAG_TOPICS="/arm/cmd_joint_velocity /panel_detect/detections /key_projector/key_positions /key_projector/panel_pose /typist/press_decisions /sim/result"
mkdir -p "$OUT"

ros_env='source /opt/ros/jazzy/setup.bash; source /opt/autotype/install/setup.bash'
codes="$(bash "$REPO/gen_codes.sh" "$RUNS")"

now() { date +%s.%N; }
since() { awk -v a="$1" -v b="$2" 'BEGIN { printf "%.1f", b - a }'; }

echo "Dashboard: http://localhost:8080   (available during each run; stack tears down on exit)"
echo "Runs: $RUNS"
echo

seed=0
pass=0
for code in $codes; do
    seed=$((seed + 1))
    echo "=================================================================="
    echo " run $seed/$RUNS   seed=$seed   launch key=$code"
    echo "=================================================================="

    # Each run's evidence is kept under its own directory; clear a stale copy.
    rundir="$OUT/$code"
    rm -rf "$rundir"
    mkdir -p "$rundir"

    t0=$(now)
    (cd "$REPO/sim" && AUTOTYPE_SEED="$seed" AUTOTYPE_LAUNCH_KEY="$code" \
        docker compose up -d --force-recreate >/dev/null 2>&1)
    t_up=$(now)
    echo "  stack up            $(since "$t0" "$t_up")s"

    # Wait for frames, not merely for the topic to exist: after a recreate,
    # discovery sometimes never completes and the stack would sit at zero.
    ready=no
    for _ in $(seq 1 25); do
        if docker exec "$DEV" bash -lc "set +u; $ros_env
            timeout 8 ros2 topic hz /camera/image_raw 2>/dev/null | grep -q 'average rate'" 2>/dev/null
        then ready=yes; break; fi
        sleep 2
    done
    t_cam=$(now)
    if [ "$ready" != yes ]; then
        echo "  SKIPPED: the simulator never started streaming ($(since "$t0" "$t_cam")s)"
        continue
    fi
    echo "  camera ready        $(since "$t_up" "$t_cam")s"

    docker exec "$DEV" bash -lc "set +u; $ros_env
        cd /ws && colcon build --symlink-install" >/dev/null 2>&1 || {
        echo "  SKIPPED: colcon build failed ($(since "$t_cam" "$(now)")s)"; continue; }
    t_build=$(now)
    echo "  build               $(since "$t_cam" "$t_build")s"

    # Record the evidence topics for the typing window. `ros2 bag record`
    # ignores a bare SIGINT when it has no tty, so it is wrapped in `timeout
    # -s INT -k`: INT finalises the bag, KILL 8 s later guarantees the wait
    # below cannot hang. The recorder gets a few extra seconds so it never
    # stops mid-episode.
    echo "  recording -> typing $code ..."
    docker exec "$DEV" bash -lc "set +u; $ros_env
        source /ws/install/setup.bash
        rm -rf /tmp/demo_${code}_bag
        timeout -s INT -k 8 $((DUR + 5)) ros2 bag record --storage mcap \
            -o /tmp/demo_${code}_bag --topics $BAG_TOPICS > /tmp/demo_bag_${code}.log 2>&1 &
        rec=\$!
        timeout -s INT -k 10 $DUR ros2 launch typist typist.launch.py > /tmp/demo_${code}.log 2>&1
        wait \$rec 2>/dev/null" >/dev/null 2>&1
    t_type=$(now)

    typed="$(docker exec "$DEV" bash -lc \
        "sed -E 's/\x1b\[[0-9;]*m//g' /tmp/demo_${code}.log | grep -o 'typed [A-Z0-9]*' | tail -1" 2>/dev/null)"
    typed="${typed#typed }"
    if [ "$typed" = "$code" ]; then
        echo "  typed $typed in $(since "$t_build" "$t_type")s  OK"
        pass=$((pass + 1))
    else
        echo "  typed '${typed:-nothing}' in $(since "$t_build" "$t_type")s  FAILED (expected $code)"
    fi

    # Persist the bag, both logs, and the converted JSONL, then report sizes.
    docker cp "$DEV:/tmp/demo_${code}_bag" "$rundir/bag" >/dev/null 2>&1
    docker cp "$DEV:/tmp/demo_${code}.log" "$rundir/$code.log" >/dev/null 2>&1
    docker cp "$DEV:/tmp/demo_bag_${code}.log" "$rundir/bag_record.log" >/dev/null 2>&1
    docker cp "$REPO/scripts/bag_to_jsonl.py" "$DEV:/tmp/bag_to_jsonl.py" >/dev/null 2>&1
    docker exec "$DEV" bash -lc "set +u; $ros_env
        source /ws/install/setup.bash
        python3 /tmp/bag_to_jsonl.py /tmp/demo_${code}_bag -o /tmp/demo_${code}.jsonl" \
        >"$rundir/convert.log" 2>&1
    docker cp "$DEV:/tmp/demo_${code}.jsonl" "$rundir/$code.jsonl" >/dev/null 2>&1
    echo "  saved $rundir  ($(du -sh "$rundir" 2>/dev/null | cut -f1), \
$(wc -l < "$rundir/$code.jsonl" 2>/dev/null || echo 0) records)"
    echo
done

echo "=================================================================="
echo " $pass of $RUNS runs typed their key exactly"
echo " Evidence under $OUT/<code>/ (bag, jsonl, launch log)"
echo "=================================================================="
