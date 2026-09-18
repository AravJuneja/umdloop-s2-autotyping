# The camera frame these came from, carrying the camera's own stamp rather
# than the time we finished detecting, so a consumer can line the corners up
# against the joint positions of the same instant.
builtin_interfaces/Time source_stamp
string frame_id

# One entry per marker found, ascending by id.
MarkerDetection[] markers

# The ids the panel carries that this frame did not show, ascending; empty
# when all four are present. Reported rather than left to be inferred: a
# consumer should not have to know which ids exist to notice one is gone.
uint16[] missing_ids
