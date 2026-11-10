std_msgs/Header header
uint8 IDLE=0
uint8 RUNNING=1
uint8 SUCCEEDED=2
uint8 CANCELED=3
uint8 FAILED=4
uint64 goal_id
uint8 state
uint8 mode
bool active
# target, progress and remaining are rad for turns, m for distance.
# Absolute-heading requests are resolved to a relative shortest-path angle.
float64 target
float64 progress
float64 remaining
float64 target_yaw
float64 heading_error
float64 command_linear_speed
float64 command_yaw_rate
float64 feedforward_gain
string message
