# Named key positions in the world frame, one entry per key of the K552.
#
# Produced by key_projector once the panel pose and key grid are located.
# Parallel arrays keep the message free of geometry_msgs: name[i] sits at
# (x[i], y[i], z[i]) in frame_id at stamp.

builtin_interfaces/Time stamp
string frame_id
string[] name
float64[] x
float64[] y
float64[] z
# Panel normal toward the arm in frame_id, from the solvePnP board fit.
float64[] normal
