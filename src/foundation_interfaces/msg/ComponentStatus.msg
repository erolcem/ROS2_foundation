# Component status message
uint8 OFFLINE = 0
uint8 ONLINE = 1
uint8 ERROR = 2
uint8 CALIBRATING = 3

string component_name
string component_type
uint8 status
string error_message
float64 last_update_time
std_msgs/Header header
