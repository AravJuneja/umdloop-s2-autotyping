#!/usr/bin/env bash
# Run what GitHub Actions runs, locally, in about twenty seconds.
#
# Both workflows, same base image, same pinned tools:
#   .github/workflows/format.yml  -> ruff format --check
#   .github/workflows/tests.yml   -> colcon build, pytest, mypy
#
# The container is created once and reused, so only the first run pays for
# installing dependencies. When tests.yml changes its apt or pip list, this
# file follows, and `make ci-clean` forces the container to be rebuilt.
#
# Usage: ./scripts/ci-local.sh        (from anywhere in the repo)
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER=umdloop-ci
IMAGE=ros:jazzy-ros-base

if ! docker info >/dev/null 2>&1; then
    echo "ERROR: docker is not running." >&2
    exit 1
fi

if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "--- creating $CONTAINER (first run only, a few minutes) ---"
    docker run -d --name "$CONTAINER" -v "$REPO:/repo" -w /repo "$IMAGE" sleep infinity >/dev/null
    # Mirrors the "Install dependencies" step of tests.yml.
    docker exec "$CONTAINER" bash -c '
        apt-get update -qq >/dev/null 2>&1
        apt-get install -y -qq --no-install-recommends \
          python3-pip python3-numpy python3-pytest \
          ros-jazzy-ament-flake8 ros-jazzy-geometry-msgs \
          ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime \
          ros-jazzy-sensor-msgs ros-jazzy-std-msgs ros-jazzy-std-srvs \
          ros-jazzy-tf2-ros ros-jazzy-tf2-ros-py >/dev/null 2>&1
        # ruff is pinned by format.yml, mypy by tests.yml; opencv from pip
        # because Ubuntu ships 4.6, which predates cv2.aruco.ArucoDetector.
        pip3 install --break-system-packages --no-cache-dir -q \
          "numpy<2" "opencv-contrib-python-headless==4.10.0.84" \
          "mypy==1.9.0" "ruff==0.16.8"' || exit 1
elif [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != true ]; then
    docker start "$CONTAINER" >/dev/null
fi

docker exec "$CONTAINER" bash -c '
set +u
FAIL=0
step() { echo; echo "### $* ###"; }

step "format: ruff format --check member_ws/src"
cd /repo && ruff format --check member_ws/src || FAIL=1

step "tests/Build: colcon build --symlink-install"
cd /repo/member_ws
. /opt/ros/jazzy/setup.sh
# Build into the container rather than into member_ws/. CI builds a fresh
# checkout, but this mounts your working tree, which usually already holds a
# build/ from the dev container -- and colcon caches absolute paths, so the
# two clobber each other and `interfaces` fails to configure. Separate bases
# keep both usable, and persist in the container so rebuilds stay fast.
if ! colcon build --symlink-install \
     --build-base /ci/build --install-base /ci/install > /tmp/build.log 2>&1; then
  echo "BUILD FAILED"; tail -25 /tmp/build.log; FAIL=1
else
  echo "build ok"
fi

step "tests/Test: pytest, one invocation per package"
. /ci/install/setup.sh
python3 -m pytest src/robot_state/test -q  || FAIL=1
python3 -m pytest src/panel_detect/test -q || FAIL=1

step "tests/Types: mypy"
python3 -m mypy --ignore-missing-imports --check-untyped-defs \
  src/robot_state/robot_state src/robot_state/test || FAIL=1
python3 -m mypy --ignore-missing-imports --check-untyped-defs \
  src/panel_detect/panel_detect src/panel_detect/test || FAIL=1

echo; echo "### RESULT: $([ $FAIL -eq 0 ] && echo PASS || echo FAIL) ###"
exit $FAIL'
