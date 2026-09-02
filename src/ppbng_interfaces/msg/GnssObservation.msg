uint8 KIND_GGA=1
uint8 KIND_UNIHEADINGA=2

std_msgs/Header header
string session_id
string device_id
uint64 sample_sequence
uint64 host_receive_monotonic_ns
uint64 connection_epoch
uint8 kind

# Absolute receiver time is populated from the validated Unicore ASCII header.
# UNKNOWN time status must leave receiver_utc_valid false.
bool receiver_utc_valid
builtin_interfaces/Time receiver_utc
string time_reference
string time_status
uint16 receiver_week
uint32 receiver_milliseconds_of_week
uint8 receiver_leap_seconds
uint16 receiver_output_delay_ms

bool gga_valid
string gga_utc_hhmmss
float64 latitude_deg
float64 longitude_deg
float64 altitude_m
int32 fix_quality
int32 satellites_used
float64 hdop
bool differential_age_valid
float64 differential_age_sec

bool heading_valid
string heading_solution_status
string heading_position_type
float64 baseline_m
float64 heading_deg
float64 pitch_deg
float64 heading_stddev_deg
float64 pitch_stddev_deg
int32 heading_satellites_tracked
int32 heading_satellites_used
