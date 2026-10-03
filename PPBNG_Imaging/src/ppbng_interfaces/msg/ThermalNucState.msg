uint8 STATE_UNKNOWN=0
uint8 STATE_IDLE=1
uint8 STATE_IN_PROGRESS=2

builtin_interfaces/Time observed_system_time
uint64 observed_host_monotonic_ns
string session_id
string device_id
uint64 state_sequence
uint8 state
bool valid
bool active
bool correction_auto_in_progress
string correction_status
string correction_status_text
string flag_state
string detail
