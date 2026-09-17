#!/usr/bin/env bash
# Launches the URC autotyping simulator (in ./sim): loads the docker images
# (downloading them first if needed) and brings up docker compose. Safe to
# re-run — it skips steps that are already done.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO/sim"

# In an uninitialized submodule, sim/ has no .git; without this, the SLUG line
# below walks up and returns THIS repo's origin, and the release download 404s.
git -C "$REPO" submodule update --init sim

# Must precede the symlink: mkdir -p through a dangling symlink fails with EEXIST.
mkdir -p "$REPO/member_ws/src"

# Recreated every run because a symlink can't be tracked in this repo (and
# breaks Windows checkouts anyway). The relative target resolves against the
# link's own directory, so it lands on the repo root.
if [[ -d "$REPO/sim/member_ws" && ! -L "$REPO/sim/member_ws" ]]; then
    echo "ERROR: sim/member_ws is a real directory, not a symlink (left over from" >&2
    echo "an older start.sh run). Its contents were never tracked." >&2
    echo "Fix: move its contents into member_ws/, run 'docker compose down', then re-run this script." >&2
    exit 1
fi
ln -sfn ../member_ws "$REPO/sim/member_ws"

ARCH=$(uname -m | sed 's/aarch64/arm64/; s/x86_64/amd64/')
if [[ "$ARCH" != "arm64" && "$ARCH" != "amd64" ]]; then
    echo "Unsupported architecture: $(uname -m)" >&2
    exit 1
fi

TARBALL="urc-autotype-$ARCH.tar.gz"
SLUG=$(git remote get-url origin | sed -E 's#(git@[^:]+:|https?://[^/]+/)##; s#\.git$##')

if ! docker image inspect urc-autotype:sim urc-autotype:dev >/dev/null 2>&1; then
    if [[ ! -f "$TARBALL" || ! -f "$TARBALL.sha256" ]]; then
        echo "Downloading $TARBALL from $SLUG releases..."
        curl -fL -O "https://github.com/$SLUG/releases/latest/download/$TARBALL"
        curl -fL -O "https://github.com/$SLUG/releases/latest/download/$TARBALL.sha256"
    fi
    echo "Verifying checksum..."
    sha256sum -c "$TARBALL.sha256"
    echo "Loading images..."
    docker load -i "$TARBALL"
else
    echo "Images already loaded, skipping download."
fi

echo "Starting containers..."
docker compose up -d

echo
echo "Simulator is up. Dashboard: http://localhost:8080"
