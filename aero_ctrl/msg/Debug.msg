# header time is set to when we send the message (update loop)
Header header

bool control_active
bool ctl_alt_changed

# bebop sync time is set when we receive a state
#time beb_sync_time

# the lag between header.stamp and beb_sync_time
float32 slm_sync_lag
float32 vcn_sync_lag
float32 fil_sync_lag
float32 ctl_fdbk_lag
float32 ctl_calc_lag
float32 beb_alt_lag
float32 beb_spd_lag

# Position and orientation of the drone from SLAM
float32 slm_x
float32 slm_y
float32 slm_z
float32 slm_roll_rad
float32 slm_pitch_rad
float32 slm_yaw_rad
float32 slm_vz

# Position and Euler angles from Vicon
float32 vcn_x
float32 vcn_y
float32 vcn_z
float32 vcn_roll_rad
float32 vcn_pitch_rad
float32 vcn_yaw_rad

# Altitude from bebop
float32 beb_alt
float32 beb_vz
float32 beb_vze

time setpoint_time
#duration setpoint_lag

# Control commands
geometry_msgs/Twist ctrl_twist

float32 Jdet
int8 sensor_fault

# For future use
#float32 thrust
#float32 torque_x
#float32 torque_y
#float32 torque_z
geometry_msgs/Pose fil_pose
geometry_msgs/Twist fil_twist
geometry_msgs/Wrench fil_wrench

geometry_msgs/Pose setpoint
