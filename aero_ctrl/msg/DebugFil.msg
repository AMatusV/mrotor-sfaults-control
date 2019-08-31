# header time is set to when we send the message
Header header

#bool prev_loop_missed

# Meas sync time is set when we receive a state
time meas_sync_time

# The lag between header.stamp and meas_sync_time
float32 meas_sync_lag
float32 fil_calc_lag
float32 delta_t

# Position and orientation of the drone from SLAM
float32 meas_x
float32 meas_y
float32 meas_z
float32 meas_qw
float32 meas_qx
float32 meas_qy
float32 meas_qz

# Mean from UKF
#float32 mean_x
#float32 mean_y
#float32 mean_z
#float32 mean_vx
#float32 mean_vy
#float32 mean_vz
#float32 mean_qw
#float32 mean_qx
#float32 mean_qy
#float32 mean_qz
#float32 mean_wx
#float32 mean_wy
#float32 mean_wz
geometry_msgs/Pose mean_pose
geometry_msgs/Twist mean_twist

geometry_msgs/Wrench mean_wrench
