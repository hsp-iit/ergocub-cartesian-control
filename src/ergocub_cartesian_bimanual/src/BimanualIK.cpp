#include <BimanualIK.h>
#include <ForwardKinematicsiDynTree.h>
#include <DifferentialInverseKinematicsQP.h>
#include <Integrator.h>
#include <stdexcept>

using Aff = Eigen::Transform<double,3,Eigen::Affine>;

static Aff make_aff(const PoseInput& p){ Aff T=Aff::Identity(); T.translate(p.pos); T.rotate(p.quat); return T; }

BimanualIK::~BimanualIK() = default;

// ctor notes:
// - joint_acc_weight: weights for accel regularization per chain [R,L,T] (size<=3)
// - joint_pos_*: size N (nR+nL+nT)
// - cart_* arrays: size 2 [right,left], allow zeros for disabled arms
// - limit_gains_rlT: gains per chain [R,L,T] in [0,1]
BimanualIK::BimanualIK(const std::string& urdf,
             const std::vector<std::string>& right_joints,
             const std::vector<std::string>& left_joints,
             const std::vector<std::string>& torso_joints,
             const std::string& right_root_frame,
             const std::string& right_ee_frame,
             const std::string& left_root_frame,
             const std::string& left_ee_frame,
             double sampling_time,
             const Eigen::VectorXd& joint_acc_weight,
             const Eigen::VectorXd& joint_pos_weights,
             const Eigen::VectorXd& joint_pos_kp,
             const Eigen::VectorXd& joint_pos_kd,
             const Eigen::Vector2d& cart_pos_weight,
             const Eigen::Vector2d& cart_pos_kp,
             const Eigen::Vector2d& cart_pos_kd,
             const Eigen::Vector2d& cart_ori_weight,
             const Eigen::Vector2d& cart_ori_kp,
             const Eigen::Vector2d& cart_ori_kd,
             double improve_manip_weight,
             const Eigen::VectorXd& q_home,
             const Eigen::VectorXd& q_lower,
             const Eigen::VectorXd& q_upper,
             const Eigen::Vector3d& limit_gains_rlT)
  : nR_((int)right_joints.size()), nL_((int)left_joints.size()), nT_((int)torso_joints.size()), N_(nR_+nL_+nT_), dt_(sampling_time)
{
  // Build per-arm joint lists including torso if provided
  std::vector<std::string> r_full=right_joints, l_full=left_joints; r_full.insert(r_full.end(), torso_joints.begin(), torso_joints.end()); l_full.insert(l_full.end(), torso_joints.begin(), torso_joints.end());
  right_fk_     = std::make_unique<ForwardKinematicsiDynTree>(urdf, r_full, right_root_frame, right_ee_frame);
  right_fk_ref_ = std::make_unique<ForwardKinematicsiDynTree>(urdf, r_full, right_root_frame, right_ee_frame);
  left_fk_      = std::make_unique<ForwardKinematicsiDynTree>(urdf, l_full, left_root_frame, left_ee_frame);
  left_fk_ref_  = std::make_unique<ForwardKinematicsiDynTree>(urdf, l_full, left_root_frame, left_ee_frame);

  ik_ = std::make_unique<DifferentialInverseKinematicsQP>(dt_, false, nR_, nL_, nT_,
    joint_acc_weight, joint_pos_weights, joint_pos_kp, joint_pos_kd,
    cart_pos_weight, cart_pos_kp, cart_pos_kd,
    cart_ori_weight, cart_ori_kp, cart_ori_kd,
    improve_manip_weight, q_home);

  Eigen::VectorXd gains(3); gains<<limit_gains_rlT(0),limit_gains_rlT(1),limit_gains_rlT(2);
  ik_->set_joint_limits(q_lower, q_upper, gains);

  acc2vel_ = std::make_unique<Integrator>(dt_);
  vel2pos_ = std::make_unique<Integrator>(dt_);
  q_=Eigen::VectorXd::Zero(N_); dq_=Eigen::VectorXd::Zero(N_); ddq_=Eigen::VectorXd::Zero(N_);
}

void BimanualIK::reset(const Eigen::VectorXd& q0, const Eigen::VectorXd& dq0){
  q_=q0; dq_ = dq0.size()==q0.size()? dq0 : Eigen::VectorXd::Zero(q0.size()); ddq_.setZero();
  acc2vel_->set_initial_condition(dq_); vel2pos_->set_initial_condition(q_);
}

Eigen::VectorXd
BimanualIK::solve_ik(const std::optional<PoseInput>& right_pose, const std::optional<PoseInput>& left_pose,
             const Eigen::Vector3d& right_lin_vel, const Eigen::Vector3d& right_ang_vel,
             const Eigen::Vector3d& left_lin_vel,  const Eigen::Vector3d& left_ang_vel,
             const Eigen::Vector3d& right_lin_acc, const Eigen::Vector3d& right_ang_acc,
             const Eigen::Vector3d& left_lin_acc,  const Eigen::Vector3d& left_ang_acc)
{
  // Current reference state (what we command)
  Eigen::VectorXd q_ref = vel2pos_->get_state();
  Eigen::VectorXd dq_ref = acc2vel_->get_state();

  // Build per-arm joint vectors (append torso when present)
  auto seg = [&](int off,int n){return q_ref.segment(off,n);};
  auto segv= [&](int off,int n){return dq_ref.segment(off,n);};
  Eigen::VectorXd r_pos, r_vel, r_acc, l_pos, l_vel, l_acc;
  if(nR_>0){ r_pos = (nT_>0)? (Eigen::VectorXd(nR_+nT_)<<seg(0,nR_),seg(nR_+nL_,nT_)).finished() : seg(0,nR_); r_vel = (nT_>0)? (Eigen::VectorXd(nR_+nT_)<<segv(0,nR_),segv(nR_+nL_,nT_)).finished() : segv(0,nR_); r_acc=Eigen::VectorXd::Zero(r_pos.size()); }
  if(nL_>0){ l_pos = (nT_>0)? (Eigen::VectorXd(nL_+nT_)<<seg(nR_,nL_),seg(nR_+nL_,nT_)).finished() : seg(nR_,nL_); l_vel = (nT_>0)? (Eigen::VectorXd(nL_+nT_)<<segv(nR_,nL_),segv(nR_+nL_,nT_)).finished() : segv(nR_,nL_); l_acc=Eigen::VectorXd::Zero(l_pos.size()); }

  // Update reference FKs with the command state
  if(nR_>0){ right_fk_ref_->set_joints_state(r_pos,r_vel,r_acc); right_fk_ref_->update(); }
  if(nL_>0){ left_fk_ref_->set_joints_state(l_pos,l_vel,l_acc);   left_fk_ref_->update(); }

  // Reference EE transforms/Jacobians/Bias acc
  Aff rT = nR_>0? right_fk_ref_->get_ee_transform(): Aff::Identity();
  Aff lT = nL_>0? left_fk_ref_->get_ee_transform(): Aff::Identity();
  Eigen::Vector3d z3=Eigen::Vector3d::Zero(); Eigen::VectorXd b0=Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd rJ = nR_>0? right_fk_ref_->get_jacobian(): Eigen::MatrixXd(6,0);
  Eigen::MatrixXd lJ = nL_>0? left_fk_ref_->get_jacobian(): Eigen::MatrixXd(6,0);
  Eigen::VectorXd rB = nR_>0? right_fk_ref_->get_ee_bias_acc(): b0; Eigen::VectorXd lB = nL_>0? left_fk_ref_->get_ee_bias_acc(): b0;
  Eigen::Vector3d rVl = nR_>0? right_fk_ref_->get_ee_lin_vel(): z3; Eigen::Vector3d rVa = nR_>0? right_fk_ref_->get_ee_ang_vel(): z3;
  Eigen::Vector3d lVl = nL_>0? left_fk_ref_->get_ee_lin_vel(): z3;  Eigen::Vector3d lVa = nL_>0? left_fk_ref_->get_ee_ang_vel(): z3;

  ik_->set_robot_state(q_ref, dq_ref, rT, rVl, rVa, right_fk_ref_->get_ee_lin_acc(), right_fk_ref_->get_ee_ang_acc(), rJ, rB,
                                   lT, lVl, lVa, left_fk_ref_->get_ee_lin_acc(),  left_fk_ref_->get_ee_ang_acc(),  lJ, lB);

  // Desired EE poses (identity if not provided)
  Aff rD=Aff::Identity(), lD=Aff::Identity();
  if(right_pose) rD = make_aff(*right_pose); if(left_pose) lD = make_aff(*left_pose);
  ik_->set_desired_ee_transform(rD,lD);
  ik_->set_desired_ee_twist(right_lin_vel,right_ang_vel,left_lin_vel,left_ang_vel);
  ik_->set_desired_ee_acceleration(right_lin_acc,right_ang_acc,left_lin_acc,left_ang_acc);

  auto ddq = ik_->solve();
  if(!ddq.has_value()) throw std::runtime_error("IK failed");
  ddq_ = ddq.value();
  acc2vel_->integrate(ddq_);
  vel2pos_->integrate(acc2vel_->get_state());
  dq_ = acc2vel_->get_state(); q_ = vel2pos_->get_state();
  return q_;
}
