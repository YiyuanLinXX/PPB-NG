# Stable identity and timing association shared by image frames and HSI lines.
# A zero camera timestamp/frame ID is meaningful only when its validity flag is true.

string session_id
string device_id
uint32 segment_id
uint64 sample_sequence

string trigger_channel
uint64 trigger_sequence
uint64 pps_sequence
uint64 controller_tick
uint64 controller_ticks_per_second

ppbng_interfaces/TimeQuality time_quality

bool camera_frame_id_valid
uint64 camera_frame_id
bool camera_timestamp_valid
uint64 camera_timestamp
uint64 host_receive_monotonic_ns

