std_msgs/Header header
# Receive timestamps and validity distinguish unknown or old data from false/zero.
builtin_interfaces/Time heartbeat_stamp
builtin_interfaces/Time status_stamp
bool heartbeat_received
bool status_received
bool online
bool enabled
uint8 work_state
uint8 mode
float64 battery_voltage
bool beep
# B1 D5 is unverified. Brake/special-function states are not available.
uint8 flags_raw
