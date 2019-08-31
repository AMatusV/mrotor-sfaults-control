#include "aero_ctrl/beb_pos_ctrl.h"


namespace beb_pos_ctrl
{

BebPosCtrl::BebPosCtrl(ros::NodeHandle &nh)
  : nh_(nh),
    nh_priv_("~"),
    nh_pid_px_(nh_priv_, "pid_forward"),
    nh_pid_py_(nh_priv_, "pid_lateral"),
    nh_pid_yaw_(nh_priv_, "pid_yaw"),
    nh_pid_alt_(nh_priv_, "pid_alt"),
    sub_ctrl_enable_(nh_.subscribe("ctrl_enable", 1, &BebPosCtrl::CtrlEnableCallback, this)),
    sub_setpoint_pose_(nh_.subscribe("setpoint_pose", 1, &BebPosCtrl::SetpointPoseCallback, this)),
    pub_ctrl_cmd_vel_(nh_.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1)),
    pub_debug_(nh_.advertise<aero_ctrl::Debug>("debug", 1)),
    sub_pose_meas_(nh_.subscribe("slam", 1, &BebPosCtrl::SlamCallback, this)),
    sub_filterP_(nh_.subscribe("pose", 1, &BebPosCtrl::FilterPCallback, this)),
    sub_filterT_(nh_.subscribe("twist", 1, &BebPosCtrl::FilterTCallback, this)),
    sub_filterW_(nh_.subscribe("wrench", 1, &BebPosCtrl::FilterWCallback, this)),
    sub_vicon_(nh_.subscribe("vicon", 1, &BebPosCtrl::ViconCallback, this)),
    sub_beb_alt_(nh_.subscribe("/bebop/states/ardrone3/PilotingState/AltitudeChanged", 1, &BebPosCtrl::BebAltCallback, this)),
    sub_beb_speed_(nh_.subscribe("/bebop/states/ardrone3/PilotingState/SpeedChanged", 1, &BebPosCtrl::BebSpeedCallback, this)),
    //subsync_beb_(BebopSyncPolicy_t(10), sub_beb_alt_, sub_beb_speed_),
    beb_param_recv_(false),
    setpoint_recv_time_(0),
    pose_recv_time_(0),
    vcn_recv_time_(0),
    fil_recv_time_(0),
    pid_px_(new control_toolbox::Pid()),
    pid_py_(new control_toolbox::Pid()),
    pid_yaw_(new control_toolbox::Pid()),
    pid_alt_(new control_toolbox::Pid()),
    ctrl_enabled_(false),
    reset_counter_(0),
    pose_recvd_(false),
    ctl_alt_changed_(false)
{
  util::GetParam(nh_priv_, "update_freq", param_update_freq_, 30.0);
  util::GetParam(nh_priv_, "min_alt", param_min_alt_, 0.4);
  util::GetParam(nh_priv_, "max_alt", param_max_alt_, 2.5);
  util::GetParam(nh_priv_, "feedback_pred_factor", param_feedback_pred_factor_, 0.2);
  util::GetParam(nh_priv_, "safety_send_zero", param_safety_send_zero_, true);
  util::GetParam(nh_priv_, "zero_xy_hover", param_xy_hover, true);

  util::GetParam(nh_priv_, "qr_mass", param_mass_, 0.4875);
  util::GetParam(nh_priv_, "qr_Ixx", param_Ixx_, 0.00223);
  util::GetParam(nh_priv_, "qr_Iyy", param_Iyy_, 0.00299);
  util::GetParam(nh_priv_, "qr_Izz", param_Izz_, 0.00480);
  util::GetParam(nh_priv_, "grav", param_grav_, 9.81);

  util::GetParam(nh_priv_, "mod_kp", param_mod_gains_[0], 1.0);
  util::GetParam(nh_priv_, "mod_ki", param_mod_gains_[1], 0.0);
  util::GetParam(nh_priv_, "mod_kd", param_mod_gains_[2], 0.0);
  
  if (!param_safety_send_zero_)
  {
    ROS_WARN("[CTL] Parameter safety_send_zero is not enabled.");
  }

  ROS_ASSERT(param_feedback_pred_factor_ > 0.0 && param_feedback_pred_factor_ <= 1.0);

  // We initialize PIDs through its nodehandle constructor,
  // The following will set some default values for the parameters if the user
  // does not specify them. This plays nice with Dynamic Reconfigure
  nh_pid_px_.setParam("p", nh_pid_px_.param("p", 0.1));
  nh_pid_px_.setParam("i", nh_pid_px_.param("i", 0.0));
  nh_pid_px_.setParam("d", nh_pid_px_.param("d", 0.01));
  nh_pid_px_.setParam("i_clamp", nh_pid_px_.param("i_clamp", 0.02));

  nh_pid_py_.setParam("p", nh_pid_py_.param("p", 0.1));
  nh_pid_py_.setParam("i", nh_pid_py_.param("i", 0.0));
  nh_pid_py_.setParam("d", nh_pid_py_.param("d", 0.01));
  nh_pid_py_.setParam("i_clamp", nh_pid_py_.param("i_clamp", 0.02));

  nh_pid_yaw_.setParam("p", nh_pid_yaw_.param("p", 0.5));
  nh_pid_yaw_.setParam("i", nh_pid_yaw_.param("i", 0.0));
  nh_pid_yaw_.setParam("d", nh_pid_yaw_.param("d", 0.01));
  nh_pid_yaw_.setParam("i_clamp", nh_pid_yaw_.param("i_clamp", 0.02));

  nh_pid_alt_.setParam("p", nh_pid_alt_.param("p", 0.3));
  nh_pid_alt_.setParam("i", nh_pid_alt_.param("i", 0.0));
  nh_pid_alt_.setParam("d", nh_pid_alt_.param("d", 0.02));
  nh_pid_alt_.setParam("i_clamp", nh_pid_alt_.param("i_clamp", 0.02));

  ROS_ASSERT(pid_px_ && pid_py_ && pid_yaw_ && pid_alt_);

  pid_px_->init(nh_pid_px_);
  pid_py_->init(nh_pid_py_);
  pid_yaw_->init(nh_pid_yaw_);
  pid_alt_->init(nh_pid_alt_);

  //subsync_beb_.registerCallback(boost::bind(&BebPosCtrl::BebSyncCallback, this, _1, _2));

  slm_pos_curr_[0] = 0.0;
  slm_pos_curr_[1] = 0.0;
  slm_pos_curr_[2] = 0.0;
  slm_ang_curr_[0] = 0.0;
  slm_ang_curr_[1] = 0.0;
  slm_ang_curr_[2] = 0.0;

  beb_z_[0] = 0.0;
  beb_z_[1] = 0.0;
  beb_z_[2] = 0.0;
  beb_vze_ = 0.0;

  beb_recv_time_ = ros::Time(0.0);
}

// AMV
void BebPosCtrl::CtrlEnableCallback(const std_msgs::Bool& enable_msg)
{
  ctrl_enabled_ = enable_msg.data;

  //double inputs[4] = {mass*grav, 0.0, 0.0, 0.0};
  //double state[12] = {0.0, 0.0, beb_alt_m_, };
  ROS_INFO_STREAM("[CTL] Enable callback: " << ctrl_enabled_);
  reset_counter_ = 0;
}

// AMV
void BebPosCtrl::SetpointPoseCallback(const geometry_msgs::PoseConstPtr& pose_ptr)
{
  setpoint_recv_time_ = ros::Time::now();
  setpoint_pose_ = *pose_ptr;
  
  ROS_INFO_STREAM("[CTL] Setpoint: " 
                  << setpoint_pose_.position.x << " "
                  << setpoint_pose_.position.y << " "
                  << setpoint_pose_.position.z << " "
                  << setpoint_pose_.orientation.z);

  // Update debug msg
  msg_debug_.setpoint_time = setpoint_recv_time_;
  msg_debug_.setpoint = setpoint_pose_;
}


void BebPosCtrl::SlamCallback(const geometry_msgs::PoseStampedConstPtr& pose_ptr)
{
  if (!beb_param_recv_) {
    // This is sketchy, I need to find a way to get these params
    if (!util::GetParam(nh_, "/bebop/bebop_driver/PilotingSettingsMaxTiltCurrent", 
                        beb_maxtilt_rad_)) {
      return;
    }
    beb_maxtilt_rad_ = angles::from_degrees(beb_maxtilt_rad_);
    if (!util::GetParam(nh_, "/bebop/bebop_driver/SpeedSettingsMaxVerticalSpeedCurrent", 
                        beb_max_speed_vert_m_)) {
      return;
    }
    if (!util::GetParam(nh_, "/bebop/bebop_driver/SpeedSettingsMaxRotationSpeedCurrent", 
                        beb_max_speed_rot_rad_)) {
      return;
    }
    beb_max_speed_rot_rad_ = angles::from_degrees(beb_max_speed_rot_rad_);

    // use first yaw as ref point
    beb_param_recv_ = true;
  }

  ros::Duration dt = ros::Time::now() - pose_recv_time_;
  if (dt.isZero()) {
    dt = ros::Duration(0.06);
  }

  pose_recv_time_ = ros::Time::now();
  pose_recvd_ = true;

  // AMV
  tf::Quaternion qc(pose_ptr->pose.orientation.x, pose_ptr->pose.orientation.y,
                    pose_ptr->pose.orientation.z, pose_ptr->pose.orientation.w);
  tf::Matrix3x3 mc(qc);
  double cam_roll_rad, cam_pitch_rad, cam_yaw_rad; 
  mc.getRPY(cam_roll_rad, cam_pitch_rad, cam_yaw_rad);

  // Camera angle transformation performed in the ORB-SLAM2 node
  // Estimate linear and angular velocities
  slm_pos_prev_[0] = slm_pos_curr_[0];  
  slm_pos_prev_[1] = slm_pos_curr_[1];
  slm_pos_prev_[2] = slm_pos_curr_[2];
  slm_ang_prev_[0] = slm_ang_curr_[0];
  slm_ang_prev_[1] = slm_ang_curr_[1];
  slm_ang_prev_[2] = slm_ang_curr_[2];
  
  slm_pos_curr_[0] = -pose_ptr->pose.position.y;
  slm_pos_curr_[1] = -pose_ptr->pose.position.x;
  slm_pos_curr_[2] = -pose_ptr->pose.position.z;
  slm_ang_curr_[0] = -cam_pitch_rad;
  slm_ang_curr_[1] = -cam_roll_rad;
  slm_ang_curr_[2] = -cam_yaw_rad;

  slm_vz_ = (slm_pos_curr_[2] - slm_pos_prev_[2])/dt.toSec();
  
  ros::Duration lag = pose_recv_time_ - pose_ptr->header.stamp;

  // Debug message update
  msg_debug_.slm_sync_lag = lag.toSec();
  //msg_debug_.beb_sync_time = pose_recv_time_;
  msg_debug_.slm_x = slm_pos_curr_[0]; 
  msg_debug_.slm_y = slm_pos_curr_[1];
  msg_debug_.slm_z = slm_pos_curr_[2];
  msg_debug_.slm_roll_rad = slm_ang_curr_[0];
  msg_debug_.slm_pitch_rad = slm_ang_curr_[1];
  msg_debug_.slm_yaw_rad = slm_ang_curr_[2];
  msg_debug_.slm_vz = slm_vz_;
}


void BebPosCtrl::FilterPCallback(const geometry_msgs::PoseStampedConstPtr& pose_ptr)
{
  msg_debug_.fil_pose = pose_ptr->pose;
}

void BebPosCtrl::FilterTCallback(const geometry_msgs::TwistStampedConstPtr& twist_ptr)
{
  msg_debug_.fil_twist = twist_ptr->twist;
}


void BebPosCtrl::FilterWCallback(const geometry_msgs::WrenchStampedConstPtr& wrench_ptr)
{
  fil_recv_time_ = ros::Time::now();

  /*if (fabs(ctrl_twist_.linear.z) < 0.01 && wrench_ptr->wrench.force.z > 0.05 && !ctl_alt_changed_) {
    pid_alt_->setGains(param_mod_gains_[0], param_mod_gains_[1], param_mod_gains_[2], 0.5, -0.5, true);
    pid_alt_->reset();
    ctl_alt_changed_ = true;
    ROS_INFO("[CTL] Altitude controller gains changed!");
  }*/

  ros::Duration lag = fil_recv_time_ - wrench_ptr->header.stamp;

  msg_debug_.fil_sync_lag = lag.toSec();
  msg_debug_.fil_wrench = wrench_ptr->wrench;
  msg_debug_.ctl_alt_changed = ctl_alt_changed_;
}


void BebPosCtrl::ViconCallback(const geometry_msgs::TransformStampedConstPtr& tf_ptr)
{
  vcn_recv_time_ = ros::Time::now();

  tf::Quaternion qv(tf_ptr->transform.rotation.x, tf_ptr->transform.rotation.y,
                    tf_ptr->transform.rotation.z, tf_ptr->transform.rotation.w);
  tf::Matrix3x3 mv(qv);
  double roll_rad, pitch_rad, yaw_rad; 
  mv.getRPY(roll_rad, pitch_rad, yaw_rad);

  ros::Duration lag = vcn_recv_time_ - tf_ptr->header.stamp;

  msg_debug_.vcn_sync_lag = lag.toSec();
	msg_debug_.vcn_x = tf_ptr->transform.translation.x;
  msg_debug_.vcn_y = tf_ptr->transform.translation.y;
  msg_debug_.vcn_z = tf_ptr->transform.translation.z;
  msg_debug_.vcn_roll_rad = roll_rad;
  msg_debug_.vcn_pitch_rad = pitch_rad;
  msg_debug_.vcn_yaw_rad = yaw_rad;
}


void BebPosCtrl::BebAltCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& alt_ptr)
{
  ros::Duration dt = ros::Time::now() - beb_recv_time_;
  if (dt.isZero()) {
    dt = ros::Duration(0.2);
  }
  beb_recv_time_ = ros::Time::now();

  beb_z_[2] = beb_z_[1];
  beb_z_[1] = beb_z_[0];
  beb_z_[0] = alt_ptr->altitude;

  beb_vze_ = (beb_z_[0] - beb_z_[1])/dt.toSec();

  ros::Duration lag = beb_recv_time_ - alt_ptr->header.stamp;
  
  msg_debug_.beb_alt_lag = lag.toSec();
	msg_debug_.beb_alt = alt_ptr->altitude;
  msg_debug_.beb_vze = beb_vze_;
}


void BebPosCtrl::BebSpeedCallback(const bebop_msgs::Ardrone3PilotingStateSpeedChangedConstPtr& speed_ptr)
{
  ros::Duration lag = ros::Time::now() - speed_ptr->header.stamp;

  msg_debug_.beb_spd_lag = lag.toSec();
  msg_debug_.beb_vz = -speed_ptr->speedZ;
}


bool BebPosCtrl::Update()
{
  ros::Time update_start = ros::Time::now();
  // This condition is already checked by the main loop for safety
  const ros::Time& t_now = ros::Time::now();
  const ros::Duration& feedback_lag = t_now - pose_recv_time_;
  //const ros::Duration& setpoint_lag = t_now - setpoint_recv_time_;
  msg_debug_.ctl_fdbk_lag = feedback_lag.toSec();
  //msg_debug_.setpoint_lag = setpoint_lag;

  if (feedback_lag.toSec() > 1.0) 
    return false;

  // AMV
  // CLAMP Input Setpoints
  setpoint_pose_.position.x = CLAMP(setpoint_pose_.position.x, -1.0, 1.0);
  setpoint_pose_.position.y = CLAMP(setpoint_pose_.position.y, -1.0, 1.0);
  setpoint_pose_.position.z = CLAMP(setpoint_pose_.position.z, -param_min_alt_, param_max_alt_);
  setpoint_pose_.orientation.z = CLAMP(setpoint_pose_.orientation.z, -3.0, 3.0);

  // PID Control Loop
  ros::Duration dt = t_now - pid_last_time_;
  ROS_INFO_STREAM("[CTL] Estim freq: " << 1.0/dt.toSec());
  /*if (dt.toSec() > (2.0 / param_update_freq_))
  {
    ROS_WARN_STREAM("PID reset: " << dt.toSec());
    pid_last_time_ = ros::Time::now();
    dt = ros::Duration(0.0);
    pid_px_->reset();
    pid_py_->reset();
    pid_alt_->reset();
    pid_yaw_->reset();
  }*/

  // AMV 
  //setpoint.orientation.z is an angle in radians
  double pos_errors[3];  // Array of position errors: x, y, z
  pos_errors[0] = setpoint_pose_.position.x - slm_pos_curr_[0];
  pos_errors[1] = setpoint_pose_.position.y - slm_pos_curr_[1];
  pos_errors[2] = setpoint_pose_.position.z - slm_pos_curr_[2];
  const double cmdX = pid_px_->computeCommand(pos_errors[0], dt);  // pitch_ref
  const double cmdY = pid_py_->computeCommand(pos_errors[1], dt);  // roll_ref
  const double vyaw_ref = pid_yaw_->computeCommand(angles::normalize_angle(
                              setpoint_pose_.orientation.z - slm_ang_curr_[2]), dt);
  double vz_ref = pid_alt_->computeCommand(pos_errors[2], dt);
 
  //TO-DO: try with beb_yaw_rad_
  double pitch_ref = sqrt(cmdX*cmdX + cmdY*cmdY)*cos(atan2(cmdY, cmdX) - slm_ang_curr_[2]);  
  double roll_ref = sqrt(cmdX*cmdX + cmdY*cmdY)*sin(atan2(cmdY, cmdX) - slm_ang_curr_[2]);

  // Position control bypass
  //const double pitch_ref = 0.0;
  //const double roll_ref = 0.0;
  //const double vz_ref = 0.0;

  // Convert PID output  into normalized cmd_vel (-1 -> 1)
  util::ResetCmdVel(ctrl_twist_);
  ctrl_twist_.linear.x =  pitch_ref / beb_maxtilt_rad_;
  ctrl_twist_.linear.y =  roll_ref / beb_maxtilt_rad_;
  ctrl_twist_.angular.z = vyaw_ref / beb_max_speed_rot_rad_;
  ctrl_twist_.linear.z = vz_ref / beb_max_speed_vert_m_;

  // CLAMP and filter output
  ctrl_twist_.linear.x = CLAMP(ctrl_twist_.linear.x, -1.0, 1.0);
  ctrl_twist_.linear.y = CLAMP(ctrl_twist_.linear.y, -1.0, 1.0);
  ctrl_twist_.linear.z = CLAMP(ctrl_twist_.linear.z, -0.25, 0.25);
  ctrl_twist_.angular.z = CLAMP(ctrl_twist_.angular.z, -1.0, 1.0);

  FILTER_SMALL_VALS(ctrl_twist_.linear.x, 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.linear.y, 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.linear.z, 0.01);
  FILTER_SMALL_VALS(ctrl_twist_.angular.z, 0.01);
  
  if (param_xy_hover && (fabs(pos_errors[0]) < 5e-2) && (fabs(pos_errors[1]) < 5e-2)) {
    ROS_INFO_ONCE("[CTL] Parameter xy_hover is enabled and the condition is met, sending vx=0, vy=0");
    ctrl_twist_.linear.x = 0.0;
    ctrl_twist_.linear.y = 0.0;
  }

  // Sensor fault detection
  double res[2], Jdet;
  res[0] = msg_debug_.beb_alt - msg_debug_.slm_z;
  //res[1] = msg_debug_.beb_vz - msg_debug_.beb_vze;
  res[1] = msg_debug_.beb_vze - msg_debug_.beb_vz;
  Jdet = sqrt(1*res[0]*res[0] + 0.5*res[1]*res[1]);

  int sensor_fault = (Jdet > 0.2) ? 1 : 0;
  msg_debug_.Jdet = Jdet;
  msg_debug_.sensor_fault = sensor_fault;
  if (sensor_fault == 1) {
    if (msg_debug_.beb_vze > 0.0)
      //ctrl_twist_.linear.z = 0.5*msg_debug_.beb_vze;
      ctrl_twist_.linear.z = ctrl_twist_.linear.z + 0.5*res[1];
    else
      //ctrl_twist_.linear.z = 0.2*msg_debug_.beb_vze;
      ctrl_twist_.linear.z = ctrl_twist_.linear.z + 0.2*res[1];
  }
    ctrl_twist_.linear.z = CLAMP(ctrl_twist_.linear.z, -0.5, 0.5);

  if (slm_pos_curr_[2] < param_min_alt_ && ctrl_twist_.linear.z < 0.0) {
    ROS_WARN_STREAM_THROTTLE(1, "[CTL] Minimum altitude safety is triggered at the altitude of " 
                             << slm_pos_curr_[2] << ". Going down is blocked.");
    ctrl_twist_.linear.z = 0.0;
  }

  if (slm_pos_curr_[2] > param_max_alt_ && ctrl_twist_.linear.z > 0.0) {
    ROS_WARN_STREAM_THROTTLE(1, "[CTL] Maximum altitude safety is triggered at the altitude of " 
                             << slm_pos_curr_[2] << ". Going up is blocked.");
    ctrl_twist_.linear.z = 0.0;
  }

  pid_last_time_ = ros::Time::now();

  pub_ctrl_cmd_vel_.publish(ctrl_twist_);

  ros::Duration lag = ros::Time::now() - update_start;

  // Update debug message
  msg_debug_.ctl_calc_lag = lag.toSec(); 
  msg_debug_.ctrl_twist = ctrl_twist_;  // AMV
  /*msg_debug_.thrust = inputs[0];
  msg_debug_.torque_x = inputs[1];
  msg_debug_.torque_y = inputs[2];
  msg_debug_.torque_z = inputs[3];*/
  pose_recvd_ = false;
  return true;
}


void BebPosCtrl::Reset()
{
  util::ResetCmdVel(ctrl_twist_);
  
  // Publish the command only once when the controller is disabled
  if (param_safety_send_zero_ && reset_counter_ < 2) {

    pub_ctrl_cmd_vel_.publish(ctrl_twist_);
  }
} 


void BebPosCtrl::Spin()
{
  ROS_INFO("[CTL] Spinnig");

  // Safety
  Reset();

  ros::Rate loop_rate(param_update_freq_);

  pid_last_time_ = ros::Time::now();
  while (ros::ok())
  {
    try
    {
      bool do_reset = false;
      bool ctrl_success = false;

      // AMV
      if (!ctrl_enabled_) {
        ROS_INFO_THROTTLE(10.0, "[CTL] Controller is not enabled.");
        do_reset = true;
        if (reset_counter_ < 255)
          reset_counter_++;
        else
          reset_counter_ = 255;
      }
      else if ((ros::Time::now() - pose_recv_time_).toSec() > 0.5) {
        ROS_WARN_THROTTLE(10.0, "[CTL] State feedback is older than 0.2 s! Resetting.");
        ctrl_enabled_ = false;  // Disable controller
        do_reset = true;
        reset_counter_ = 1;
      }

      if (do_reset)
      {
        Reset();
      }
      else if (pose_recvd_)
      {
        ctrl_success = Update();
      }

      msg_debug_.control_active = ctrl_success;
      msg_debug_.header.stamp = ros::Time::now();
      pub_debug_.publish(msg_debug_);

      ros::spinOnce();
      if (!loop_rate.sleep())
      {
        ROS_WARN_STREAM("[CTL] Missed loop update rate of " << param_update_freq_);
      }
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR_STREAM("[CTL] Runtime Exception: " << e.what());
      Reset();
    }
  }
}

}  // namespace beb_pos_ctrl
