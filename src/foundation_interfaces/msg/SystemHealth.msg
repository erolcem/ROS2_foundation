# System health monitoring message
float64 overall_health_score
ComponentStatus[] component_statuses
string[] warnings
string[] errors
geometry_msgs/Vector3 temperature  # x: CPU, y: GPU, z: ambient
std_msgs/Header header
