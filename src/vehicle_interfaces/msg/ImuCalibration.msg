uint8 DISABLED=0
uint8 WAITING=1
uint8 SETTLING=2
uint8 POSITIVE=3
uint8 PAUSE=4
uint8 NEGATIVE=5
uint8 VERIFYING=6
uint8 SUCCEEDED=7
uint8 FAILED=8
uint8 CANCELED=9
std_msgs/Header header
uint8 state
bool active
# True after at least one successful calibration in this driver process.
bool calibrated
# Effective multiplier applied to mounted raw IMU yaw, never to imu/data itself.
int8 yaw_sign
float64 positive_delta
float64 negative_delta
float64 command_yaw_rate
string message
