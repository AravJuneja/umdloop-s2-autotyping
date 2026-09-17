# robot_state

Normalizes the simulator's raw topics into one timestamped view of the robot,
so the rest of our code never has to ask whether a message is recent, whether
its arrays line up, or which order the joints came in.

## Running it

```bash
ros2 launch robot_state robot_state.launch.py   # or: ros2 run robot_state robot_state
```

It logs a one-line health summary for every input every five seconds.

## Reading the state

Three ways in, all backed by the same snapshots:

- `~/updates/<input>` — latched per-input topic, published whenever that input
  changes or its health changes. A late subscriber gets the current value
  immediately rather than waiting for the next message.
- `~/get_state` — a `GetRobotState` service returning all six inputs aged
  against a single clock reading, so their ages are comparable to each other.
- `StateNode.on_update(callback)` — in-process, for a node composed with this
  one. Each consumer gets its own copy.

The node also runs a normal TF listener, so `tf_buffer` answers real lookups
across the tree rather than just the transforms we happened to receive.

## What a status means

Every observation carries an `ObservationStatus` with three independent
answers, which is the point of the whole package:

- `arrived` — has anything ever come in on this topic.
- `fresh` — is what arrived recent enough to act on. Measured from the
  source's own stamp when there is one, so a message delayed in transit is
  not mistaken for current data. Latched inputs are always fresh.
- `valid` — was the data well-formed, with `problems` listing what was wrong.

Rate health is reported separately in `rate_known`, `rate_hz`, and `rate_ok`,
and deliberately does not feed `valid`: a publisher running slow does not make
the numbers it sent wrong.

A stale value is kept, not discarded. A consumer deciding whether to act on
old data needs to see the data.

## Changing the contract

Every threshold lives in `robot_state/config.py` — topics, QoS, freshness
windows, expected rates, and the tolerances applied to quaternions and camera
matrices. Adding an input means adding one `InputSpec` and one branch in
`normalize`; nothing else needs to know.
