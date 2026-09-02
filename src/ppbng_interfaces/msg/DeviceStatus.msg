# Per-device state. Device lifecycle and health are orthogonal: a STREAMING
# device can be DEGRADED while the global acquisition remains RECORDING.

uint8 LIFECYCLE_DISCONNECTED=0
uint8 LIFECYCLE_DISCOVERED=1
uint8 LIFECYCLE_OPEN=2
uint8 LIFECYCLE_CONFIGURED=3
uint8 LIFECYCLE_ARMED=4
uint8 LIFECYCLE_STREAMING=5
uint8 LIFECYCLE_RECOVERING=6
uint8 LIFECYCLE_STOPPING=7

uint8 HEALTH_UNKNOWN=0
uint8 HEALTH_OK=1
uint8 HEALTH_DEGRADED=2
uint8 HEALTH_FAULT=3

builtin_interfaces/Time status_time
uint64 status_host_monotonic_ns
string session_id
string device_id
bool required
uint8 lifecycle_state
uint8 health
uint32 segment_id

bool last_sample_valid
uint64 last_sample_sequence
builtin_interfaces/Time last_sample_utc_time
uint64 last_sample_host_monotonic_ns

uint64 samples_received
uint64 samples_incomplete
uint64 samples_lost
uint64 samples_dropped
uint32 reconnect_attempts
string active_config_hash
string[] actual_setting_keys
string[] actual_setting_values
string detail
