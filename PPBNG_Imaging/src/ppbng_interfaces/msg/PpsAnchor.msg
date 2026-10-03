std_msgs/Header header

uint8 UNSYNCED=0
uint8 LOCKED=1
uint8 HOLDOVER=2

uint32 boot_id
uint64 pps_sequence
uint64 captured_tick
uint64 host_receive_monotonic_ns
uint8 lock

# When utc_valid is false, utc_second MUST be INT64_MIN. The timing controller
# does not parse GNSS and must never present its unresolved PPS as valid UTC.
bool utc_valid
int64 utc_second
