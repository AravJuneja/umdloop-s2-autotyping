string source
bool arrived
bool has_source_stamp
builtin_interfaces/Time source_stamp
builtin_interfaces/Time received_stamp
float64 age_sec -1.0
float64 received_age_sec -1.0
# True for inputs published once and valid for the rest of the episode;
# freshness_sec carries no window for those. -1.0 on the age fields means
# "not computed", a meaning freshness_sec no longer shares.
bool latched
float64 freshness_sec
bool fresh
bool valid
string[] problems
bool rate_known
float64 rate_hz
float64 expected_rate_hz
bool rate_ok
