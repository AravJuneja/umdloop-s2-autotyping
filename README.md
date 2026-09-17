This repo contains code for software challenge 2 specified in [this doc](challenge_spec.pdf)

Our Logs can be found [here](https://docs.google.com/document/d/1WLQU7h1sxwaj7nFyXOctIzjUhHK8tUPtBf6354ASlB4/edit?usp=sharing)

## Layout

- `sim/` — the URC autotyping simulator, vendored in as a git submodule
  ([AutoTypingChallengeSim](https://github.com/Rishav-N/AutoTypingChallengeSim)).
  Don't edit anything under here — it's someone else's repo, checked out at a
  fixed commit.
- `member_ws/` — our actual code: the ROS 2 workspace for the typist node
  (`member_ws/src/...`). `member_ws/src/` is tracked in this repo; build
  output (`build/`, `install/`, `log/`) is gitignored. Two packages so far:
  `robot_state`, which normalizes the simulator's topics into one timestamped
  view of the robot (see its own README), and `robot_state_interfaces`, which
  holds the messages and the `GetRobotState` service it publishes.
- `start.sh` — launches the simulator (downloads/loads the docker images on
  first run, then `docker compose up -d`). Run it from this directory.

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
   ros2 run my_typist typist
   ```
5. Watch it act on the sim at http://localhost:8080.
6. Commit and push from this repo as usual — `member_ws/src/` is regular
   tracked content here, `sim/` stays untouched at its pinned submodule
   commit.

Note: `sim/docs/INFO.md` refers to paths like
`~/AutoTypingChallengeSim/member_ws/...` — in this repo those resolve through
the `sim/member_ws` symlink to `member_ws/` here.

See `sim/docs/INFO.md` for the full simulator setup/reference and
`sim/docs/INTERFACES.md` for the topic/message reference.
