#!/usr/bin/env bash
# Launches the URC autotyping simulator (in ./sim): loads the docker images
# (downloading them first if needed) and brings up docker compose. Safe to
# re-run — it skips steps that are already done.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/sim"

ARCH=$(uname -m | sed 's/aarch64/arm64/; s/x86_64/amd64/')
if [[ "$ARCH" != "arm64" && "$ARCH" != "amd64" ]]; then
    echo "Unsupported architecture: $(uname -m)" >&2
    exit 1
fi

TARBALL="urc-autotype-$ARCH.tar.gz"
SLUG=$(git remote get-url origin | sed -E 's#(git@[^:]+:|https?://[^/]+/)##; s#\.git$##')

if ! docker image inspect urc-autotype:sim urc-autotype:dev >/dev/null 2>&1; then
    if [[ ! -f "$TARBALL" ]]; then
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

mkdir -p member_ws/src

echo "Starting containers..."
docker compose up -d

echo
echo "Simulator is up. Dashboard: http://localhost:8080"
