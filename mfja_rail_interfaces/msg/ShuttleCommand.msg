std_msgs/Header header
string name
string command
string start_slot
float64 speed
# Optional closed-loop motion setpoint. Empty preserves continuous ON motion.
# The rail controller resolves this against its authoritative device config;
# it is not a visual-model prediction or a localization observation.
string target_slot
