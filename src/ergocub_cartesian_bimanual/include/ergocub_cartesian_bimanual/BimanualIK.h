// Minimal YARP-free bimanual IK façade.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <tuple>

class ForwardKinematicsiDynTree;
class DifferentialInverseKinematicsQP;
class Integrator;

// Cartesian pose input: position (m) and quaternion (xyzw)
struct PoseInput { Eigen::Vector3d pos; Eigen::Quaterniond quat; };

class BimanualIK {
public:
  BimanualIK(const std::string& urdf,
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
             const Eigen::Vector2d& cart_pos_weight,   // [right,left]
             const Eigen::Vector2d& cart_pos_kp,       // [right,left]
             const Eigen::Vector2d& cart_pos_kd,       // [right,left]
             const Eigen::Vector2d& cart_ori_weight,   // [right,left]
             const Eigen::Vector2d& cart_ori_kp,       // [right,left]
             const Eigen::Vector2d& cart_ori_kd,       // [right,left]
             double improve_manip_weight,
             const Eigen::VectorXd& q_home,
             const Eigen::VectorXd& q_lower,
             const Eigen::VectorXd& q_upper,
             const Eigen::Vector3d& limit_gains_rlT);

  ~BimanualIK();

  void reset(const Eigen::VectorXd& q0, const Eigen::VectorXd& dq0 = Eigen::VectorXd());

  // One control step: returns q after one IK + integration.
  Eigen::VectorXd
  solve_ik(const std::optional<PoseInput>& right_pose,
           const std::optional<PoseInput>& left_pose,
           const Eigen::Vector3d& right_lin_vel = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d& right_ang_vel = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d& left_lin_vel  = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d& left_ang_vel  = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d& right_lin_acc = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d& right_ang_acc = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d& left_lin_acc  = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d& left_ang_acc  = Eigen::Vector3d::Zero());

private:
  int nR_{0}, nL_{0}, nT_{0}, N_{0};
  double dt_;
  // FK of measured/reference models per arm
  std::unique_ptr<ForwardKinematicsiDynTree> right_fk_, right_fk_ref_;
  std::unique_ptr<ForwardKinematicsiDynTree> left_fk_,  left_fk_ref_;
  std::unique_ptr<DifferentialInverseKinematicsQP> ik_;
  std::unique_ptr<Integrator> acc2vel_, vel2pos_;
  Eigen::VectorXd q_, dq_, ddq_;
};
