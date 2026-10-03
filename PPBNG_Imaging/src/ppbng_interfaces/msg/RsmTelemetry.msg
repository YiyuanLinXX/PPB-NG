builtin_interfaces/Time status_time
uint64 host_receive_monotonic_ns
string session_id
string device_id
uint32 segment_id
uint64 sample_sequence

bool roll_valid
float64 roll_deg
bool pitch_valid
float64 pitch_deg
bool yaw_valid
float64 yaw_deg

bool timer_valid
int64 mount_timer_ms
bool general_status_valid
int32 control_source
int32 major_status
int32 motion_status
int32 built_in_test_status
int32 error_level
bool error_groups_valid
uint16[4] error_bits

string raw_frame
string validation_detail
