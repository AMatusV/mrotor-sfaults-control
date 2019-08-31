#ifndef FILTER_EXT
#define FILTER_EXT

#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/Bool.h>
#include <fstream>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include "aero_ctrl/DebugFil.h"


namespace filter_ext
{

// Global constants that define our state vector size
const int QS_SZ = 19;
const int ST_SZ = 18;
const int MS_SZ = 7;
const int VN_SZ = 12;
const int WN_SZ = 6;
const int IN_SZ = 4;

namespace util {

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

class FilterExt
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Subscriber sub_enable_;
  ros::Subscriber sub_meas_;

  ros::Publisher pub_pose_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_wrench_;
  ros::Publisher pub_debug_;

  int stackedSize, noisesSize;
  size_t sigmaCount;

  // Message objects
  geometry_msgs::PoseStamped pose_msg_;
  geometry_msgs::TwistStamped twist_msg_;
  geometry_msgs::WrenchStamped wrench_msg_;
  aero_ctrl::DebugFil msg_debug_;

  // Filter parameters
  double alpha, beta, kappa, lambda, param_update_freq_;
  bool fil_enabled_, meas_recvd_;

  // Vectors
  Eigen::Matrix<double,QS_SZ,1> state, qSigmaPt;
  Eigen::Matrix<double,ST_SZ,1> stateMR;
  Eigen::Matrix<double,MS_SZ,1> meas;
  Eigen::Matrix<double,WN_SZ,1> measMR;
  Eigen::Matrix<double,IN_SZ,1> controlInput;
  Eigen::Vector4d q_km1;

  // Matrices
  Eigen::Matrix<double,ST_SZ,ST_SZ> stateCovar;
  Eigen::Matrix<double,VN_SZ,VN_SZ> processCovar;
  Eigen::Matrix<double,WN_SZ,WN_SZ> measCovar; 

  // UKF sigma points and related values
  std::vector<double> weights;
  std::vector<Eigen::VectorXd> sigmaPts, qPredSigmaPts, xPredSigmaPts, zPredSigmaPts;
  Eigen::MatrixXd weightedCovarSqrt, N;
  Eigen::MatrixXd PPred;
  Eigen::MatrixXd PxzPred;
  Eigen::MatrixXd S;

  // Time variables
  ros::Duration delta_t;
  ros::Time meas_recv_time_, meas_prev_time_, fil_calc_time_;

  // Physical quadrotor parameters
  double grav, mass;
  Eigen::Matrix3d inertia;

  // Xml values for initialization
  XmlRpc::XmlRpcValue initStaConf, initStaCovConf, procCovConf, measCovConf; 

  void EnableCallback(const std_msgs::Bool& enableMsg);
  void MeasCallback(const geometry_msgs::PoseStampedConstPtr& pose_ptr);

  Eigen::Vector4d quatmultiply(const Eigen::Vector4d& q, const Eigen::Vector4d& r);
  Eigen::Vector4d quatinv(const Eigen::Vector4d& q);
  Eigen::Vector4d mrp2errquat(const Eigen::Vector3d& delta_rho);
  Eigen::Vector3d errquat2mrp(const Eigen::Vector4d& error_quaternion);
  Eigen::Matrix3d hatmap(const Eigen::Vector3d& vector);
  Eigen::Matrix4d OmegaFunc(const Eigen::Vector3d& omega_k, double T);
  Eigen::Matrix3d quat2rotm(const Eigen::Vector4d& quaternion);
  Eigen::Matrix<double,QS_SZ,1> quadrotorModel(const Eigen::Matrix<double,QS_SZ,1>& state_km1, const Eigen::Vector4d& input, const Eigen::Matrix<double,VN_SZ,1>& noise, double T);
  Eigen::Matrix<double,WN_SZ,1> obsModel(const Eigen::Matrix<double,ST_SZ,1>& sta, const Eigen::Matrix<double,WN_SZ,1>& noise);
  void modelProp(size_t i);
  void UKF_Calcs(void);
  Eigen::MatrixXd LoadMatrix(const Eigen::MatrixXd& mat, XmlRpc::XmlRpcValue vals, int sz, bool is_vec);

public:
  FilterExt(ros::NodeHandle& nh);
  
  virtual void Spin();
};

}  // namespace filter_ext

#endif  // FILTER_EXT
