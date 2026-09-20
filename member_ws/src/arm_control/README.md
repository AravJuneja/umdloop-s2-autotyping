# arm_control

Drives the arm to a joint-space pose and holds it there, closed loop against
`robot_state`'s normalized joint feed. It is the only publisher of
`/arm/cmd_joint_velocity` -- callers ask for a pose, never a velocity, and
this node knows nothing about keys or panels.

## Asking for a pose

`~/move_joints` (`interfaces/action/MoveJoints`) is the only way in.

```
string[] name       # every one of the five known joints, any order
float64[] position  # rad, parallel to name
---
string state        # terminal: SETTLED, STOPPED, or FAULTED
string[] name
float64[] position
---
string state        # MOVING while active
string[] name
float64[] position
float64[] velocity
float64[] error      # position - target
```

A goal is rejected outright (no result, no feedback) if it does not name each
of the five joints exactly once, if any target falls outside that joint's
limits (`sim/docs/INTERFACES.md` section 3.1), or if the joint feed is not
currently healthy. Sending a new goal preempts whatever goal is running;
canceling one stops the arm in place. Either way the preempted/canceled goal
finishes with `state = STOPPED`.

## What "reached" means

A goal finishes `SETTLED` once every joint is simultaneously:

- within **0.01 rad (about 0.57°)** of its target, and
- moving at no more than **0.02 rad/s** (the number issue #5 states),

held continuously for **0.3 s** before being declared settled -- a single
sample under both thresholds could be noise passing through the target, not
an arrival. Both constants live in `arm_control/config.py`
(`POSITION_TOLERANCE_RAD`, `SETTLE_DWELL_SEC`); change them there, not here.

## Reading the state without a goal

`~/status` (`interfaces/msg/ArmStatus`, latched) reports `MOVING`, `SETTLED`,
`STOPPED`, or `FAULTED` at all times, whether or not a goal is currently
active -- the same latched-per-change convention `robot_state` uses for
`~/updates/<input>`, so a late subscriber sees the current state immediately.

`FAULTED` means the joint feed from `robot_state` went stale or invalid; this
node reacts by publishing an explicit zero velocity to all five joints (never
a partial stop -- an omitted joint keeps its last commanded velocity per
`INTERFACES.md` 8.5) and, if a goal was active, ending it with
`state = FAULTED`. It does not retry on its own; the next goal a caller sends
once the feed recovers is what moves the arm again.

## Control loop

One proportional term per joint, `v = clip(kp * (target - position), -v_max,
v_max)`, run at 50 Hz while a goal is active. `kp` is chosen per joint so
every joint saturates at its own `v_max` beyond the same 0.2 rad error and
decelerates smoothly inside it; the simulator's own actuator already ramps
toward whatever velocity is commanded within `a_max`
(`INTERFACES.md` 3.4), so nothing here reproduces that ramp.

The loop publishes on every tick it runs, at 50 Hz -- an order of magnitude
inside the simulator's 0.10 s command watchdog
(`INTERFACES.md` 8.5) -- and stops publishing once a goal ends (settled,
canceled, preempted, or faulted), after one final explicit all-zero command.
With nothing left to correct, the arm stays where it was left; if this
process dies mid-motion, the watchdog is what stops the arm, not this node.

## Running it

```bash
ros2 run arm_control arm_control
```

Depends on `robot_state` already running and publishing
`/robot_state/updates/joints`.

## Changing the contract

Joint order, limits, gains, and the settle thresholds all live in
`arm_control/config.py`. The control law itself (`arm_control/control.py`) is
pure -- no ROS, no clock -- so it is tested by handing it numbers
(`test/test_control.py`) rather than standing up a node.
