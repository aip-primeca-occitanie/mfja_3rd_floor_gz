std_msgs/Header header
string name
string command
string start_slot
# Configured travel speed for ON only. OFF/RESET/REMOVE publishers set zero;
# the controller retains its internal ON speed setting across OFF.
float64 speed
# Optional closed-loop motion setpoint. Empty preserves continuous ON motion.
# The rail controller resolves this against its authoritative device config;
# it is not a visual-model prediction or a localization observation.
string target_slot
