#ifndef BEB_POS_PID
#define BEB_POS_PID

#include <boost/shared_ptr.hpp>
#include <control_toolbox/pid.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
// AMV
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateSpeedChanged.h>

// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

#include "aero_ctrl/Debug.h"

#ifndef CLAMP
#define CLAMP(x, l, h) (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))
#endif

#ifndef FILTER_SMALL_VALS
#define FILTER_SMALL_VALS(x, eps) (x = ((fabs((x)) < (eps)) ? 0.0 : (x)))
#endif


namespace beb_pos_pid
{

namespace util {

inline void ResetCmdVel(geometry_msgs::Twist& v)
{
  v.linear.x = 0.0;
  v.linear.y = 0.0;
  v.linear.z = 0.0;
  v.angular.x = 0.0;
  v.angular.y = 0.0;
  v.angular.z = 0.0;
}

template<typename T>
bool GetParam(const ros::NodeHandle& nh, const::std::string& key, T& val)
{
  if (nh.getParam(key, val))
  {
    ROS_INFO_STREAM("[CTL] Param " << key << " : " << val);
    return true;
  }
  ROS_WARN_STREAM("[CTL] Param " << key << " not found/set.");
  return false;
}

template<typename T>
bool GetParam(const ros::NodeHandle& nh, const::std::string& key, T& val, const T& default_val)
{
  nh.param(key, val, default_val);
  ROS_INFO_STREAM("[CTL] Param " << key << " : " << val);
}

}  // namespace util


// typedef message_filters::sync_policies::ApproximateTime<
//   bebop_msgs::Ardrone3PilotingStateAltitudeChanged,
//   bebop_msgs::Ardrone3PilotingStateSpeedChanged> BebopSyncPolicy_t;


class BebPosPid
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // This is required for proper initialization of PID dynamic reconfigure
  ros::NodeHandle nh_pid_px_;
  ros::NodeHandle nh_pid_py_;
  ros::NodeHandle nh_pid_yaw_;
  ros::NodeHandle nh_pid_alt_;

  ros::Subscriber sub_ctrl_enable_;
  ros::Subscriber sub_setpoint_pose_;
  ros::Subscriber sub_pose_meas_;
  ros::Subscriber sub_filterP_;
  ros::Subscriber sub_filterT_;  
  ros::Subscriber sub_filterW_; 
  ros::Subscriber sub_vicon_;

  ros::Subscriber sub_beb_alt_;
  ros::Subscriber sub_beb_speed_;

/*   message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateAltitudeChanged> sub_beb_alt_;
  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateSpeedChanged> sub_beb_speed_;
  message_filters::Synchronizer<BebopSyncPolicy_t> subsync_beb_; */

  ros::Publisher pub_ctrl_cmd_vel_;
  ros::Publisher pub_debug_;

  geometry_msgs::Pose setpoint_pose_;
  geometry_msgs::Twist ctrl_twist_;
  //geometry_msgs::Twist ctrl_twist_prev_;

  // Bebop internal params (driver reads/sets them)
  bool beb_param_recv_;
  double beb_maxtilt_rad_;
  double beb_max_speed_vert_m_;
  double beb_max_speed_rot_rad_;

  // Params
  double param_update_freq_;
  bool param_safety_send_zero_;
  bool param_xy_hover;
  double param_min_alt_;
  double param_max_alt_;
  double param_feedback_pred_factor_;
  double param_mod_gains_[3];

  double param_mass_;
  double param_grav_;
  double param_Ixx_;
  double param_Iyy_;
  double param_Izz_;

  ros::Time setpoint_recv_time_;
  ros::Time pose_recv_time_;
  ros::Time vcn_recv_time_;
  ros::Time fil_recv_time_;
  //ros::Time pose_last_time_;
  
  // Slam States
  double slm_pos_prev_[3], slm_ang_prev_[3];
  double slm_pos_curr_[3], slm_ang_curr_[3];
  double slm_vz_;

  // PID Controllers
  ros::Time pid_last_time_;
  boost::shared_ptr<control_toolbox::Pid> pid_px_;
  boost::shared_ptr<control_toolbox::Pid> pid_py_;
  boost::shared_ptr<control_toolbox::Pid> pid_yaw_;
  boost::shared_ptr<control_toolbox::Pid> pid_alt_;

  aero_ctrl::Debug msg_debug_;

  // AMV
  bool ctrl_enabled_;
  unsigned char reset_counter_;
  /*double roll_cmd_[2];
  double pitch_cmd_[2];
  double vyaw_cmd_[2];
  double vz_cmd_[2];
  double roll_deriv_[2];
  double pitch_deriv_[2];*/
  bool pose_recvd_;
  bool ctl_alt_changed_;

  ros::Time beb_recv_time_;
  double beb_z_[3], beb_vz_;

  void SlamCallback(const geometry_msgs::PoseStampedConstPtr& pose_ptr);
  void FilterPCallback(const geometry_msgs::PoseStampedConstPtr& pose_ptr);
  void FilterTCallback(const geometry_msgs::TwistStampedConstPtr& twist_ptr);
  void FilterWCallback(const geometry_msgs::WrenchStampedConstPtr& wrench_ptr);
  void ViconCallback(const geometry_msgs::TransformStampedConstPtr& tf_ptr);

  void BebAltCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& alt_ptr);
  void BebSpeedCallback(const bebop_msgs::Ardrone3PilotingStateSpeedChangedConstPtr& speed_ptr);
  
/*   void BebSyncCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& alt_ptr,
                         const bebop_msgs::Ardrone3PilotingStateSpeedChangedConstPtr& speed_ptr); */

  void CtrlEnableCallback(const std_msgs::Bool& enable_msg);  // AMV
  void SetpointPoseCallback(const geometry_msgs::PoseConstPtr& pose_ptr);

  bool Update();

public:
  BebPosPid(ros::NodeHandle& nh);

  void Reset();
  virtual void Spin();
};

}  // namespace beb_pos_pid

#endif  // BEB_POS_pid
