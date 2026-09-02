# Structured, stable fault event. Human-readable detail supplements fault_code;
# software must not parse detail to make decisions.

uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARNING=1
uint8 SEVERITY_ERROR=2
uint8 SEVERITY_FATAL=3

string event_id
string session_id
string source_id
string fault_code
uint8 severity

bool utc_time_valid
builtin_interfaces/Time first_utc_time
builtin_interfaces/Time last_utc_time
uint64 first_host_monotonic_ns
uint64 last_host_monotonic_ns
uint64 occurrence_count

bool latched
bool acknowledged
bool causes_global_stop
string detail

