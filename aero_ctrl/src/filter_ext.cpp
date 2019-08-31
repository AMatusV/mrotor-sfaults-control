#include "aero_ctrl/filter_ext.h"


namespace filter_ext
{

FilterExt::FilterExt(ros::NodeHandle &nh)
  : nh_(nh),
    nh_priv_("~"),
    sub_enable_(nh_.subscribe("enable", 1, &FilterExt::EnableCallback, this)),
    sub_meas_(nh_.subscribe("meas", 1, &FilterExt::MeasCallback, this)),
    pub_pose_(nh_.advertise<geometry_msgs::PoseStamped>("pose", 1)),
    pub_twist_(nh_.advertise<geometry_msgs::TwistStamped>("twist", 1)),
    pub_wrench_(nh_.advertise<geometry_msgs::WrenchStamped>("wrench", 1)),
    pub_debug_(nh_.advertise<aero_ctrl::DebugFil>("debug", 1)),
    meas_recv_time_(0),
    meas_prev_time_(0),
    fil_calc_time_(0),
    fil_enabled_(false),
    meas_recvd_(false)
{
  util::GetParam(nh_priv_, "update_freq", param_update_freq_, 100.0);
  util::GetParam(nh_priv_, "alpha", alpha, 1.0);
  util::GetParam(nh_priv_, "beta", beta, 0.0);
  util::GetParam(nh_priv_, "kappa", kappa, 2.0);

  inertia.setZero();
  util::GetParam(nh_priv_, "qr_mass", mass, 0.4875);
  util::GetParam(nh_priv_, "qr_Ixx", inertia(0,0), 0.00223);
  util::GetParam(nh_priv_, "qr_Iyy", inertia(1,1), 0.00299);
  util::GetParam(nh_priv_, "qr_Izz", inertia(2,2), 0.00480);
  util::GetParam(nh_priv_, "grav", grav, 9.81);

  util::GetParam(nh_priv_, "init_sta", initStaConf);
  util::GetParam(nh_priv_, "init_sta_cov", initStaCovConf);
  util::GetParam(nh_priv_, "proc_cov", procCovConf);
  util::GetParam(nh_priv_, "meas_cov", measCovConf);

  ROS_ASSERT(initStaConf.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(initStaCovConf.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(procCovConf.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(measCovConf.getType() == XmlRpc::XmlRpcValue::TypeArray);

  state = LoadMatrix(state, initStaConf, QS_SZ, true);  
  stateCovar = LoadMatrix(stateCovar, initStaCovConf, ST_SZ, false);
  processCovar = LoadMatrix(processCovar, procCovConf, VN_SZ, false);
  measCovar = LoadMatrix(measCovar, measCovConf, WN_SZ, false);

  controlInput.setZero();
  controlInput(0) = mass*grav;

  meas.setZero();
  meas(3) = 1.0;

  // Calculate the dimensions of the problem
  stackedSize = ST_SZ + VN_SZ + WN_SZ;
  noisesSize = VN_SZ + WN_SZ;

  // Number of sigma points and scaling terms
  sigmaCount = (stackedSize << 1) + 1;

  // Compute lambda according to scaling parameters
  lambda = alpha*alpha*(stackedSize + kappa) - stackedSize;

  // Allocate space
  weights.resize(sigmaCount + 1);  // +1 to save the zeroth covariance term
  sigmaPts.resize(sigmaCount, Eigen::VectorXd(stackedSize));
  qPredSigmaPts.resize(sigmaCount, Eigen::VectorXd(QS_SZ));
  xPredSigmaPts.resize(sigmaCount, Eigen::VectorXd(ST_SZ));
  zPredSigmaPts.resize(sigmaCount, Eigen::VectorXd(WN_SZ));

  weights[0] = lambda/(stackedSize + lambda);
  sigmaPts[0].setZero();
  qPredSigmaPts[0].setZero();
  xPredSigmaPts[0].setZero();
  zPredSigmaPts[0].setZero();
  for (size_t i = 1; i < sigmaCount; ++i) {
    sigmaPts[i].setZero();
	  qPredSigmaPts[i].setZero();
	  xPredSigmaPts[i].setZero();
	  zPredSigmaPts[i].setZero();
    weights[i] =  1.0 / (2.0 * (stackedSize + lambda));
  }
  weights[sigmaCount] = weights[0] + (1.0 - alpha*alpha + beta);  // zeroth covariance term

  N.resize(noisesSize, noisesSize);
  N << processCovar, Eigen::MatrixXd::Zero(VN_SZ,WN_SZ), Eigen::MatrixXd::Zero(WN_SZ,VN_SZ), measCovar;

  std::cout << "Initial state" << std::endl << state << std::endl;
  std::cout << "Initial covariance" << std::endl << stateCovar << std::endl;
  std::cout << "Process covariance" << std::endl << processCovar << std::endl;
  std::cout << "Measurement covariance" << std::endl << measCovar << std::endl;
}


Eigen::MatrixXd FilterExt::LoadMatrix(const Eigen::MatrixXd& mat, XmlRpc::XmlRpcValue vals, int sz, bool is_vec)
{
  Eigen::MatrixXd res(mat);
  res.setZero();

  if (is_vec) {
    for (int i = 0; i < sz; i++) {
      std::ostringstream ostr;
			ostr << vals[i];
			std::istringstream istr(ostr.str());
			istr >> res(i);
    }
  }
  else {
    for (int i = 0; i < sz; i++) {
      std::ostringstream ostr;
			ostr << vals[i];
			std::istringstream istr(ostr.str());
			istr >> res(i,i);
    } 
  }
  return res;
}


void FilterExt::EnableCallback(const std_msgs::Bool& enable_msg)
{
  fil_enabled_ = enable_msg.data;

  ROS_INFO_STREAM("[FIL] Enable callback: " << fil_enabled_);
}


void FilterExt::MeasCallback(const geometry_msgs::PoseStampedConstPtr& pose_ptr)
{
  meas_prev_time_ = meas_recv_time_;
  meas_recv_time_ = ros::Time::now();

  if (!meas_prev_time_.isZero()) {
    delta_t = meas_recv_time_ - meas_prev_time_;
    if (delta_t.isZero()) {
      ROS_WARN_STREAM("[FIL] Time step is 0, skipping this loop at time " << meas_recv_time_);
      return;
    }
    meas_recvd_ = true;
  }
  else {
	  state(0) = pose_ptr->pose.orientation.w;
	  state(1) = - pose_ptr->pose.orientation.y; 
	  state(2) = - pose_ptr->pose.orientation.x;
	  state(3) = - pose_ptr->pose.orientation.z;
	  state(7) = - pose_ptr->pose.position.y;
	  state(8) = - pose_ptr->pose.position.x;
	  state(9) = - pose_ptr->pose.position.z;
	  msg_debug_.mean_pose.position.x = state(7);
    msg_debug_.mean_pose.position.y = state(8);
    msg_debug_.mean_pose.position.z = state(9);
    msg_debug_.mean_pose.orientation.w = state(0);
    msg_debug_.mean_pose.orientation.x = state(1);
    msg_debug_.mean_pose.orientation.y = state(2);
    msg_debug_.mean_pose.orientation.z = state(3);
	  ROS_INFO("[FIL] Prev time is 0, doing nothing.");
	  std::cout << "State changed to" << std::endl << state << std::endl;
    return;
  }

  meas(0) = - pose_ptr->pose.position.y;
  meas(1) = - pose_ptr->pose.position.x;
  meas(2) = - pose_ptr->pose.position.z;
  meas(3) = pose_ptr->pose.orientation.w;  // transform quaternion
  meas(4) = - pose_ptr->pose.orientation.y;  // rot pi about x, rot pi/2 about z
  meas(5) = - pose_ptr->pose.orientation.x;
  meas(6) = - pose_ptr->pose.orientation.z;

  ros::Duration lag = meas_recv_time_ - pose_ptr->header.stamp;

  // Debug message update
  msg_debug_.meas_sync_time = meas_recv_time_;
  msg_debug_.meas_sync_lag = lag.toSec();  // not significant when using bags
  msg_debug_.delta_t = delta_t.toSec();
  msg_debug_.meas_x = meas(0);
  msg_debug_.meas_y = meas(1);
  msg_debug_.meas_z = meas(2);
  msg_debug_.meas_qw = meas(3);
  msg_debug_.meas_qx = meas(4);
  msg_debug_.meas_qy = meas(5);
  msg_debug_.meas_qz = meas(6);
}


Eigen::Vector4d FilterExt::quatmultiply(const Eigen::Vector4d& q, const Eigen::Vector4d& r)
{
  Eigen::Vector4d n;
	
  n(0) = r(0)*q(0) - r(1)*q(1) - r(2)*q(2) - r(3)*q(3);
  n(1) = r(0)*q(1) + r(1)*q(0) - r(2)*q(3) + r(3)*q(2);
  n(2) = r(0)*q(2) + r(1)*q(3) + r(2)*q(0) - r(3)*q(1);
  n(3) = r(0)*q(3) - r(1)*q(2) + r(2)*q(1) + r(3)*q(0);

  return n;
}


Eigen::Vector4d FilterExt::quatinv(const Eigen::Vector4d& q)
{
  Eigen::Vector4d inv_q;
  double norm_squared = q.squaredNorm(); 
  inv_q(0) = q(0)/norm_squared;
  inv_q(1) = -q(1)/norm_squared;
  inv_q(2) = -q(2)/norm_squared;
  inv_q(3) = -q(3)/norm_squared;

  return inv_q;
}


Eigen::Vector4d FilterExt::mrp2errquat(const Eigen::Vector3d& delta_rho)
{
  double delta_q_0;
  //Eigen::Vector3d delta_q_v;
  delta_q_0 = (1.0 - delta_rho.transpose()*delta_rho) /  (1.0 + delta_rho.transpose()*delta_rho);
  //delta_q_v = delta_rho*(1.0 + delta_q_0);

  Eigen::Vector4d errorQuat;
  //errorQuat << delta_q_0, delta_q_v;
  errorQuat << delta_q_0, delta_rho*(1.0 + delta_q_0);

  return errorQuat;
}


Eigen::Vector3d FilterExt::errquat2mrp(const Eigen::Vector4d& error_quat)
{
  double delta_q_0;
  //Eigen::Vector3d delta_q_v;	
  //delta_q_0 = error_quat(0);
  //delta_q_v = error_quat.tail(3);

  Eigen::Vector3d delta_rho;
  //delta_rho = delta_q_v/(1.0 + delta_q_0);
  delta_rho = error_quat.tail(3)/(1.0 + error_quat(0));

  return delta_rho;
}


Eigen::Matrix3d FilterExt::hatmap(const Eigen::Vector3d& vector)
{
  Eigen::Matrix3d matrix;
  matrix << 0.0, -vector(2), vector(1),
            vector(2), 0.0, -vector(0),
   	       -vector(1), vector(0), 0.0;
	
  return matrix;
}


Eigen::Matrix4d FilterExt::OmegaFunc(const Eigen::Vector3d& omega_k, double T)
{
  Eigen::Vector3d phi_k;

  if (omega_k.norm() < std::numeric_limits<double>::epsilon()) {
	  phi_k << 0.0, 0.0, 0.0;	
  }
  else {
    phi_k = sin(0.5*omega_k.norm()*T)*omega_k/omega_k.norm();
  }

  //Eigen::Matrix3d phicross_k;
  //phicross_k = hatmap(phi_k);
	
  Eigen::Matrix4d omg;
  //omg << cos(0.5*omega_k.norm()*T), -phi_k.transpose(), phi_k, cos(0.5*omega_k.norm()*T)*Eigen::MatrixXd::Identity(3,3) + phicross_k;
  omg << cos(0.5*omega_k.norm()*T), -phi_k.transpose(), phi_k, cos(0.5*omega_k.norm()*T)*Eigen::MatrixXd::Identity(3,3) + hatmap(phi_k);

  return omg; 
}


Eigen::Matrix3d FilterExt::quat2rotm(const Eigen::Vector4d& quat)
{
  //double q_0;
  //Eigen::Vector3d q_v;
  //Eigen::Matrix3d qcross_v;
  //q_0 = quat(0);
  //q_v = quat.tail(3);
  //qcross_v = hatmap(q_v);

  Eigen::Matrix3d R_transpose;
  //R_transpose = (2.0*q_0*q_0 - 1.0)*Eigen::MatrixXd::Identity(3,3) + 2.0*q_v*q_v.transpose() - 2.0*q_0*qcross_v;
  R_transpose = (2.0*quat(0)*quat(0) - 1.0)*Eigen::MatrixXd::Identity(3,3) + 2.0*quat.tail(3)*quat.tail(3).transpose() - 2.0*quat(0)*hatmap(quat.tail(3));

  return R_transpose;
}


Eigen::Matrix<double,QS_SZ,1> FilterExt::quadrotorModel(const Eigen::Matrix<double,QS_SZ,1>& state_km1, const Eigen::Vector4d& input, const Eigen::Matrix<double,VN_SZ,1>& noise, double T)
{
  // Gravity vector
  Eigen::Vector3d g_vec(0.0, 0.0, grav);	

  // Renaming the state variables
  Eigen::Vector4d q_km1;
  Eigen::Vector3d omega_km1, x_km1, xdot_km1, taue_km1, fe_km1;
  q_km1 = state_km1.head(4);
  omega_km1 = state_km1.segment(4, 3);
  x_km1 = state_km1.segment(7, 3);
  xdot_km1 = state_km1.segment(10, 3);
  taue_km1 = state_km1.segment(13, 3);
  fe_km1 = state_km1.tail(3);

  // Renaming inputs
  Eigen::Vector3d taum_km1(input.tail(3));
  Eigen::Vector3d ct_km1(0.0, 0.0, input(0));	

  // Renaming noise variables
  Eigen::Vector3d etatm_k, etate_k, etact_k, etafe_k;
  etatm_k = noise.head(3);
  etate_k = noise.segment(3, 3);
  etact_k = noise.segment(6, 3);
  etafe_k = noise.tail(3);

  // Compute the prior rotation matrix
  Eigen::Matrix3d R_km1;
  R_km1 = quat2rotm(q_km1);

  // Translational dynamics
  Eigen::Vector3d xddot_km1, x_k, xdot_k;
  xddot_km1 = R_km1.transpose()*(ct_km1 + etact_k)/mass - g_vec + fe_km1/mass;
  x_k = x_km1 + T*xdot_km1 + 0.5*T*T*xddot_km1;
  xdot_k = xdot_km1 + T*xddot_km1;

  // Rotational dynamics
  Eigen::Vector4d q_k;
  Eigen::Vector3d omega_k;
  q_k = OmegaFunc(omega_km1, T)*q_km1;
  omega_k = omega_km1 + T*inertia.inverse()*(R_km1*taue_km1 + taum_km1 + etatm_k - omega_km1.cross(inertia*omega_km1));

  // Dynamics of external perturbations	
  Eigen::Vector3d taue_k, fe_k;
  taue_k = taue_km1 + etate_k;
  fe_k = fe_km1 + etafe_k;

  Eigen::Matrix<double,QS_SZ,1> state_k;
  state_k << q_k, omega_k, x_k, xdot_k, taue_k, fe_k;

  return state_k;
}


Eigen::Matrix<double,WN_SZ,1> FilterExt::obsModel(const Eigen::Matrix<double,ST_SZ,1>& sta, const Eigen::Matrix<double,WN_SZ,1>& noise)
{
  //Eigen::Vector3d x, delta_rho, eta_x, eta_rho;
  //x = sta.segment(6,3);
  //delta_rho = sta.head(3);
  //eta_x = noise.head(3);
  //eta_rho = noise.tail(3);	

  Eigen::Matrix<double,WN_SZ,1> obs;
  //obs.head(3) = x + eta_x;
  //obs.tail(3) = delta_rho + eta_rho;
  obs.head(3) = sta.segment(6,3) + noise.head(3);
  obs.tail(3) = sta.head(3) + noise.tail(3);

  return obs;
}


void FilterExt::modelProp(size_t i)
{
  qSigmaPt << quatmultiply(mrp2errquat(sigmaPts[i].head(3)), q_km1), sigmaPts[i].segment(3,15); 
  qPredSigmaPts[i] = quadrotorModel(qSigmaPt, controlInput, sigmaPts[i].segment(ST_SZ,VN_SZ), delta_t.toSec());	
  xPredSigmaPts[i] << errquat2mrp(quatmultiply(qPredSigmaPts[i].head(4), quatinv(qPredSigmaPts[0].head(4)))), qPredSigmaPts[i].tail(QS_SZ-4);
  zPredSigmaPts[i] = obsModel(xPredSigmaPts[i], sigmaPts[i].tail(WN_SZ));
}


void FilterExt::UKF_Calcs()
{
  fil_calc_time_ = ros::Time::now();

  // Save previous state quaternion
  q_km1 = state.head(4);

  // Save observation quaternion
  Eigen::Vector4d q_obs;
  q_obs = meas.tail(4);

  // Augment state with noise vectors.	
  Eigen::MatrixXd PQ(stackedSize, stackedSize);
  Eigen::VectorXd xQ(ST_SZ + noisesSize);
  PQ << stateCovar, Eigen::MatrixXd::Zero(ST_SZ,noisesSize), Eigen::MatrixXd::Zero(noisesSize,ST_SZ), N;
  xQ << 0.0, 0.0, 0.0, state.tail(15), Eigen::MatrixXd::Zero(noisesSize,1); 

  // (1) Take the square root of the stacked covariance matrix
  weightedCovarSqrt = ((stackedSize + lambda)*PQ).llt().matrixL();

  // (2) Compute sigma points and pass them through the function
  sigmaPts[0] = xQ;
  modelProp(0);	
  for (size_t sigmaInd = 0; sigmaInd < stackedSize; ++sigmaInd) {
    sigmaPts[sigmaInd + 1] = xQ - weightedCovarSqrt.col(sigmaInd);
	  modelProp(sigmaInd + 1);
    sigmaPts[sigmaInd + 1 + stackedSize] = xQ + weightedCovarSqrt.col(sigmaInd);
	  modelProp(sigmaInd + 1 + stackedSize);
  }

  // (3) Sum the weighted sigma points to generate a new state prediction
  stateMR.setZero();
  measMR.setZero();
  for (size_t sigmaInd = 1; sigmaInd < sigmaPts.size(); ++sigmaInd) {
	  stateMR.noalias() += weights[sigmaInd]*(xPredSigmaPts[sigmaInd] - xPredSigmaPts[0]);
	  measMR.noalias() += weights[sigmaInd]*(zPredSigmaPts[sigmaInd] - zPredSigmaPts[0]);
  }
  stateMR = stateMR + xPredSigmaPts[0];
  measMR = measMR + zPredSigmaPts[0];

  //msg_debug_.pred_wx = stateMR(3);
  //msg_debug_.pred_wy = stateMR(4);
  //msg_debug_.pred_wz = stateMR(5);

  // Convert the mean perturbation to error quaternion then to quaternion
  Eigen::Matrix<double,QS_SZ,1> sPred;
  Eigen::Matrix<double,MS_SZ,1> yPred;
  sPred << quatmultiply(mrp2errquat(stateMR.head(3)), qPredSigmaPts[0].head(4)), stateMR.tail(ST_SZ-3);
  yPred << measMR.head(3), quatmultiply(mrp2errquat(measMR.tail(3)), qPredSigmaPts[0].head(4));

  // (4) Use the sigma points and the predicted state to compute a predicted covariance
  Eigen::Matrix<double,ST_SZ,1> exSigmaPt;
  Eigen::Matrix<double,WN_SZ,1> ezSigmaPt;
  exSigmaPt = xPredSigmaPts[0] - stateMR;
  ezSigmaPt = zPredSigmaPts[0] - measMR;
	
  // Work out the covariances and the cross correlations. Note that the weigth on the 0th
  // point is different from the mean calculation due to scaled unscented algorithm
  PPred = weights[sigmaCount]*exSigmaPt*exSigmaPt.transpose();
  PxzPred = weights[sigmaCount]*exSigmaPt*ezSigmaPt.transpose();
  S = weights[sigmaCount]*ezSigmaPt*ezSigmaPt.transpose();
  for (size_t sigmaInd = 1; sigmaInd < sigmaPts.size(); ++sigmaInd) {
	  exSigmaPt = xPredSigmaPts[sigmaInd] - stateMR;
	  ezSigmaPt = zPredSigmaPts[sigmaInd] - measMR;
	  PPred.noalias() += weights[sigmaInd]*exSigmaPt*exSigmaPt.transpose();
	  PxzPred.noalias() += weights[sigmaInd]*exSigmaPt*ezSigmaPt.transpose();
	  S.noalias() += weights[sigmaInd]*ezSigmaPt*ezSigmaPt.transpose();
  }

  // Calculate Kalman gain
  Eigen::Matrix<double,ST_SZ,WN_SZ> K;
  K = PxzPred*S.inverse();

  // Compare the measurement with the predicted one and then convert to MRP
  Eigen::Matrix<double,WN_SZ,1> convMeas;
  convMeas << meas.head(3), errquat2mrp(quatmultiply(q_obs, quatinv(yPred.tail(4))));
	
  // Calculate innovation
  Eigen::Matrix<double,WN_SZ,1> innovation;
  Eigen::Matrix<double,ST_SZ,1> dState;
  innovation = convMeas - measMR;
  dState = K*innovation;

  // Update mean
  state << quatmultiply(mrp2errquat(dState.head(3)), sPred.head(4)), sPred.tail(QS_SZ-4) + dState.tail(ST_SZ-3);
	
  // Update covariance
  stateCovar = PPred - K*S*K.transpose();

  pose_msg_.header.stamp = meas_recv_time_;
  pose_msg_.pose.orientation.w = state(0);
  pose_msg_.pose.orientation.x = state(1);
  pose_msg_.pose.orientation.y = state(2);
  pose_msg_.pose.orientation.z = state(3);
  pose_msg_.pose.position.x = state(7);
  pose_msg_.pose.position.y = state(8);
  pose_msg_.pose.position.z = state(9);

  twist_msg_.header.stamp = meas_recv_time_;
  twist_msg_.twist.angular.x = state(4);
  twist_msg_.twist.angular.y = state(5);
  twist_msg_.twist.angular.z = state(6);
  twist_msg_.twist.linear.x = state(10);
  twist_msg_.twist.linear.y = state(11);
  twist_msg_.twist.linear.z = state(12);

  wrench_msg_.header.stamp = meas_recv_time_;
  wrench_msg_.wrench.torque.x = state(13);
  wrench_msg_.wrench.torque.y = state(14);
  wrench_msg_.wrench.torque.z = state(15);
  wrench_msg_.wrench.force.x = state(16);
  wrench_msg_.wrench.force.y = state(17);
  wrench_msg_.wrench.force.z = state(18);

  pub_pose_.publish(pose_msg_);
  pub_twist_.publish(twist_msg_);
  pub_wrench_.publish(wrench_msg_);

  meas_recvd_ = false;

  ros::Duration calc_lag = ros::Time::now() - fil_calc_time_;

  // Debug message update
  msg_debug_.fil_calc_lag = calc_lag.toSec();
  msg_debug_.mean_pose = pose_msg_.pose;
  msg_debug_.mean_twist = twist_msg_.twist;
  msg_debug_.mean_wrench = wrench_msg_.wrench;
}


void FilterExt::Spin()
{
  ROS_INFO("[FIL] Spinnig");

  ros::Rate loop_rate(param_update_freq_);

  while (ros::ok()) {
    try {
      if (!fil_enabled_) {
        state.head(4) = meas.tail(4);
		    state.segment(7,3) = meas.head(3);
        ROS_INFO_THROTTLE(10, "[FIL] Filter is not enabled.");
      }
      else if (meas_recvd_) {
        UKF_Calcs();
      } 

      //msg_debug_.header.stamp = ros::Time::now();
      //pub_debug_.publish(msg_debug_);

      ros::spinOnce();
      if (!loop_rate.sleep()) {
        ROS_WARN_STREAM("[FIL] Missed loop update rate of " << param_update_freq_);
      }
    }
    catch (const std::runtime_error& e) {
      ROS_ERROR_STREAM("[FIL] Runtime Exception: " << e.what());
    }
  }
}

}  // namespace filter_ext