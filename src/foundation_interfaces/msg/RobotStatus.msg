# Robot status message
uint8 UNKNOWN = 0
uint8 INITIALIZING = 1
uint8 READY = 2
uint8 RUNNING = 3
uint8 ERROR = 4
uint8 EMERGENCY_STOP = 5

uint8 status
string robot_id
string[] active_components
float64 battery_level
float64 cpu_usage
float64 memory_usage
std_msgs/Header header
