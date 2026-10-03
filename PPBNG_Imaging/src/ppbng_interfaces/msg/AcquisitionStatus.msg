uint8 IDLE=0
uint8 PREFLIGHT=1
uint8 WAITING_FOR_DARK=2
uint8 CAPTURING_DARK=3
uint8 WAITING_FOR_SAMPLE=4
uint8 WAITING_FOR_PPS=5
uint8 RECORDING=6
uint8 STOPPING=7
uint8 FINALIZED=8
uint8 FAULT=9

std_msgs/Header header
uint8 state
bool degraded
string session_id
string session_directory
string[] active_devices
string[] warnings
string detail

