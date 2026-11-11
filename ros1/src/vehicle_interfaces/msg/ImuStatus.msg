std_msgs/Header header
bool online
bool measurement_valid
bool orientation_valid
uint8 nmt_state
uint8 measurement_status
uint8 orientation_status
uint32 faults
uint32 sample_count
uint32 quaternion_count
float64 measurement_age
float64 orientation_age
uint32 orientation_device_age_ms
uint64 rx_frames
uint64 malformed_frames
uint64 error_frames
uint32 dropped_frames
string message
# classic or legacy_fd; waiting until detected when imu_protocol=auto.
string protocol
# Legacy status/faults/counters/device-age fields above are usable only when true.
bool device_metadata_available
bool heartbeat_valid
bool acceleration_valid
bool angular_velocity_valid
bool magnetic_field_valid
# Host received PDO totals, never device sample counters.
uint64 received_measurements
uint64 received_orientations
