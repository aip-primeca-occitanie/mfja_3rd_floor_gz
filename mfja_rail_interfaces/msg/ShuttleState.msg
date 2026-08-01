std_msgs/Header header
string name
string mode
string current_segment
float64 s
float64 x
float64 y
float64 z
float64 yaw
float64 speed
# Low-level actuation completion only. Empty unless the controller stopped at
# an explicit ShuttleCommand.target_slot setpoint.
string reached_target_slot
