# arm_control's control state, independent of whether a goal is active right
# now. Published on ~/status latched (transient-local), so a late subscriber
# -- a dashboard, a panel-aiming node checking the arm is idle before it asks
# for a new pose -- sees the current value immediately rather than waiting for
# the next transition, the same convention robot_state uses for ~/updates/<input>.

string MOVING=moving      # driving toward target, outside tolerance or too fast to call settled
string SETTLED=settled    # within tolerance and under the velocity threshold, holding
string STOPPED=stopped    # commanding zero velocity with no goal active (startup, cancel, done)
string FAULTED=faulted    # joint-state input is stale or invalid; commanding zero, goal rejected

builtin_interfaces/Time stamp
string state
string[] name
float64[] position
float64[] velocity
# The pose being held or driven toward. Equals position at STOPPED/FAULTED
# with no prior goal -- there is nothing to hold yet.
float64[] target
# Why FAULTED, e.g. "joint_states not fresh"; empty at every other state.
string[] problems
