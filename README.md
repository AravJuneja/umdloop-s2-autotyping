This repo contains code for software challenge 2 specified in [this doc](challenge_spec.pdf)

Our Logs can be found [here](https://docs.google.com/document/d/1WLQU7h1sxwaj7nFyXOctIzjUhHK8tUPtBf6354ASlB4/edit?usp=sharing)

## Layout

- `sim/` — the URC autotyping simulator, vendored in as a git submodule
  ([AutoTypingChallengeSim](https://github.com/Rishav-N/AutoTypingChallengeSim)).
  Don't edit anything under here — it's someone else's repo, checked out at a
  fixed commit.
- `member_ws/` — our actual code: the ROS 2 workspace for the typist node
  (`member_ws/src/...`). `member_ws/src/` is tracked in this repo; build
  output (`build/`, `install/`, `log/`) is gitignored. Five packages:
  `robot_state`, which normalizes the simulator's topics into one timestamped
  view of the robot; `panel_detect`, which finds the panel's four ArUco
  markers, estimates the panel pose, and projects world-frame key positions;
  `arm_control`, which drives the arm to joint-space poses closed loop;
  `typist`, which solves one pose per launch-key character and types it;
  and `interfaces`, which holds every message, service, and action this
  workspace defines. The first three have their own READMEs.
- `start.sh` — launches the simulator (downloads/loads the docker images on
  first run, then `docker compose up -d`). Run it from this directory.
- `Makefile` / `e2e.sh` — self-contained one-line runners, no new files
  needed. Everything runs inside the `dev` container (which mounts this
  workspace at `/ws` via the `sim/member_ws` symlink), so each target is a
  single host command:
  - `make up` — start sim + dev (idempotent; safe to re-run).
  - `make e2e` — full end-to-end loop: build the workspace, launch the
    typing stack, and type 3 seeds, checking `/sim/result` for an exact
    match each episode. `make e2e SEEDS=20` for a longer loop.
  - `make check` — repo tests (pytest, mirrors CI).
  - `make logs` — tail the typing stack's output. `make down` — stop it.

## Workflow

0. `git clone --recurse-submodules <url>` (or run `git submodule update --init`
   after a plain clone — `start.sh` does this for you).
1. `./start.sh` — brings up the `sim` and `dev` containers and the dashboard
   at http://localhost:8080. On first run it also initializes the `sim`
   submodule and creates the `sim/member_ws` symlink (below), so a fresh
   clone needs nothing else. Set `AUTOTYPE_PORT` to use a different port
   (e.g. `AUTOTYPE_PORT=8091 ./start.sh`), such as when running a second,
   isolated stack alongside the first — see `sim/docker-compose.yml` for
   details.
2. `sim/member_ws` is a symlink to `./member_ws` in this repo, created by
   `start.sh` on every run (a symlink can't be tracked in git). Docker's bind
   mount (`sim`'s `docker-compose.yml` maps `./member_ws` to `/ws` in the
   `dev` container) follows that symlink, so `/ws` inside the container and
   `member_ws/` here are the same files.
3. Write and edit the ROS 2 package under `member_ws/src/` — either directly
   on the host, or via VS Code's "Dev Containers: Attach to Running
   Container" for in-container editing with ROS 2 tooling available.
4. Build and run it inside the `dev` container:
   ```bash
   cd sim && docker compose exec dev bash
   cd /ws && colcon build --symlink-install
   source /ws/install/setup.bash
   ros2 launch typist typist.launch.py
   ```
   Or in one line from the repo root: `make e2e` (builds, launches the
   stack, and types 3 seeds, checking each `/sim/result` for an exact
   match; `make e2e SEEDS=20` for a longer loop).
5. Watch it act on the sim at http://localhost:8080.
6. Commit and push from this repo as usual — `member_ws/src/` is regular
   tracked content here, `sim/` stays untouched at its pinned submodule
   commit.

## Python Formatting

Install the pinned formatter in your Python environment and run it from the repo root:

```bash
python -m pip install ruff==0.16.8
ruff format member_ws/src
ruff format --check member_ws/src
```

Ruff uses single quotes and a 99-character line length. Only Python files under
`member_ws/src/` are in scope; `sim/` is excluded. The separate `format` CI workflow
checks formatting without replacing the existing lint and type checks.

Note: `sim/docs/INFO.md` refers to paths like
`~/AutoTypingChallengeSim/member_ws/...` — in this repo those resolve through
the `sim/member_ws` symlink to `member_ws/` here.

See `sim/docs/INFO.md` for the full simulator setup/reference and
`sim/docs/INTERFACES.md` for the topic/message reference.

## Checklist

- [x] Read the launch key from /sim/launch_key
- [x] Control the arm in closed loop from /joint_states, within its joint and velocity limits.
- [x] Determine the panel's position and orientation from the camera image.
- [x] Determine where the keys of the launch key are. Key positions are not provided
- [x] Move the arm to a pose from which every character of the launch key can be pressed
- [x] Aim at and press each character in order, then publish /sim/done
- [x] type the launch key exactly, so /sim/result reports an exact match
- [x] work on any episode seed without code changes. The panel's placement changes between episodes.
- [ ] record detections, pose estimates, commands, and press decisions in an exportable log 
