.PHONY: up e2e e2e-codes down logs check test ci ci-clean stats demo

# Self-contained one-liners. Everything runs through the `dev` container,
# which already has ROS, the sim messages, and this workspace mounted at
# /ws via start.sh's sim/member_ws symlink -- so no new files are ever
# needed on the host. Each target is one command:
#
#   make up          # start sim + dev (idempotent; safe to re-run)
#   make e2e         # full loop: build, launch stack, type 3 seeds (default)
#   make e2e SEEDS=20
#   make e2e-codes  # type fresh random codes, one sim container per code
#   make e2e-codes CODES=20
#   make ci          # everything GitHub runs: format, build, pytest, mypy
#   make ci-clean    # discard the ci container so the next run rebuilds it
#   make stats       # time the stack over N random codes (make stats N=10)
#   make demo        # watchable: N runs, new seed and key each, dashboard up
#   make check       # pytest only -- NOT the full gate; see `make ci`
#   make logs        # tail the typing stack's output
#   make down        # stop the stack
SEEDS ?= 3
CODES ?= 3
N ?= 5

up:
	./start.sh

down:
	cd sim && docker compose down

logs:
	cd sim && docker compose exec dev bash -lc 'tail -f /tmp/typist.log'

# The full CI gate. `check` below runs only the pytest half, so it can pass
# while GitHub fails on format or mypy -- prefer this before pushing.
ci:
	./scripts/ci-local.sh

ci-clean:
	-docker rm -f umdloop-ci

stats:
	./scripts/e2e-stats.sh $(N)

# Like stats, but on the default port so the dashboard stays at :8080, and
# the stack is left running at the end instead of torn down.
demo:
	./start.sh
	./scripts/demo.sh $(N)

check:
	cd sim && docker compose exec -T dev bash -lc '\
	  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
	  cd /ws && python3 -m pytest src/robot_state/test -q && \
	  python3 -m pytest src/panel_detect/test -q'

test:
	cd sim && docker compose exec -T dev bash -lc '\
	  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
	  cd /ws && python3 -m pytest src/robot_state/test -q && \
	  python3 -m pytest src/panel_detect/test -q'

e2e:
	./start.sh
	cd sim && docker compose cp ../e2e.sh dev:/tmp/e2e.sh && docker compose exec -T dev bash -lc '\
	  source /opt/ros/jazzy/setup.bash && \
	  source /ws/install/setup.bash 2>/dev/null || true && \
	  cd /ws && colcon build --symlink-install && \
	  source /ws/install/setup.bash && \
	  (nohup ros2 launch typist typist.launch.py > /tmp/typist.log 2>&1 &) && \
	  sleep 5 && \
	  SEEDS="$(SEEDS)" bash /tmp/e2e.sh'

e2e-codes:
	./start.sh
	cd sim && docker compose cp ../e2e_codes.sh dev:/tmp/e2e_codes.sh && docker compose cp ../gen_codes.sh dev:/tmp/gen_codes.sh && docker compose exec -T -e CODES="$(CODES)" dev bash -lc '\
	  source /opt/ros/jazzy/setup.bash && \
	  source /ws/install/setup.bash 2>/dev/null || true && \
	  cd /ws && colcon build --symlink-install && \
	  source /ws/install/setup.bash && \
	  (nohup ros2 launch typist typist.launch.py > /tmp/typist.log 2>&1 &) && \
	  sleep 5 && \
	  bash /tmp/e2e_codes.sh dev'
