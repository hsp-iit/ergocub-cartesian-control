/*
 * Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * MIT license. See the accompanying LICENSE file for details.
 */

#include <ergocub_cartesian_bimanual/ForwardKinematicsPinocchio.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <stdexcept>

using namespace Eigen;

namespace
{
bool is_single_dof_joint(const pinocchio::Model& model, const pinocchio::JointIndex joint_idx)
{
    return joint_idx > 0 &&
           joint_idx < static_cast<pinocchio::JointIndex>(model.njoints) &&
           model.nqs[joint_idx] == 1 &&
           model.nvs[joint_idx] == 1;
}
}

ForwardKinematicsPinocchio::ForwardKinematicsPinocchio(const std::string& urdf_path, const std::vector<std::string>& joints_list, const std::string& root_frame_name, const std::string& ee_frame_name)
{
    try
    {
        pinocchio::urdf::buildModel(urdf_path, model_);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error(log_prefix_ + "::ctor. Cannot load model from " + urdf_path + ": " + e.what());
    }

    data_ = std::make_unique<pinocchio::Data>(model_);
    joints_list_ = joints_list;

    root_frame_idx_ = model_.getFrameId(root_frame_name);
    ee_frame_idx_ = model_.getFrameId(ee_frame_name);

    if (root_frame_idx_ >= model_.nframes)
    {
        throw std::runtime_error(log_prefix_ + "::ctor. Cannot find root frame " + root_frame_name + " in model " + urdf_path);
    }
    if (ee_frame_idx_ >= model_.nframes)
    {
        throw std::runtime_error(log_prefix_ + "::ctor. Cannot find end-effector frame " + ee_frame_name + " in model " + urdf_path);
    }

    joint_bindings_.reserve(joints_list.size());
    for (std::size_t i = 0; i < joints_list.size(); ++i)
    {
        const auto& joint_name = joints_list[i];
        if (!model_.existJointName(joint_name))
        {
            throw std::runtime_error(log_prefix_ + "::ctor. Cannot find joint " + joint_name + " in model " + urdf_path);
        }

        const pinocchio::JointIndex joint_idx = model_.getJointId(joint_name);
        if (!is_single_dof_joint(model_, joint_idx))
        {
            throw std::runtime_error(log_prefix_ + "::ctor. Joint " + joint_name + " is not a single-DoF Pinocchio joint.");
        }

        joint_bindings_.push_back({static_cast<int>(i), model_.idx_qs[joint_idx], model_.idx_vs[joint_idx]});
    }

    q_ = pinocchio::neutral(model_);
    dq_ = VectorXd::Zero(model_.nv);
    ddq_ = VectorXd::Zero(model_.nv);
    zero_ddq_ = VectorXd::Zero(model_.nv);
    full_jacobian_ = MatrixXd::Zero(6, model_.nv);
    jacobian_ = MatrixXd::Zero(6, static_cast<int>(joint_bindings_.size()));
    bias_acc_ = VectorXd::Zero(6);

    transform_.setIdentity();
    lin_vel_.setZero();
    ang_vel_.setZero();
    lin_acc_.setZero();
    ang_acc_.setZero();
}

void ForwardKinematicsPinocchio::update()
{
    pinocchio::forwardKinematics(model_, *data_, q_, dq_, ddq_);
    pinocchio::updateFramePlacements(model_, *data_);
    pinocchio::computeJointJacobians(model_, *data_);

    const pinocchio::SE3 root_H_ee = data_->oMf[root_frame_idx_].inverse() * data_->oMf[ee_frame_idx_];
    transform_.linear() = root_H_ee.rotation();
    transform_.translation() = root_H_ee.translation();

    const Matrix3d world_R_root = data_->oMf[root_frame_idx_].rotation();
    full_jacobian_.setZero();
    pinocchio::getFrameJacobian(model_, *data_, ee_frame_idx_, pinocchio::LOCAL_WORLD_ALIGNED, full_jacobian_);

    for (std::size_t i = 0; i < joint_bindings_.size(); ++i)
    {
        jacobian_.col(static_cast<int>(i)).topRows<3>() = world_R_root.transpose() * full_jacobian_.col(joint_bindings_[i].v_index).topRows<3>();
        jacobian_.col(static_cast<int>(i)).bottomRows<3>() = world_R_root.transpose() * full_jacobian_.col(joint_bindings_[i].v_index).bottomRows<3>();
    }

    const auto velocity = pinocchio::getFrameVelocity(model_, *data_, ee_frame_idx_, pinocchio::LOCAL_WORLD_ALIGNED);
    const auto acceleration = pinocchio::getFrameClassicalAcceleration(model_, *data_, ee_frame_idx_, pinocchio::LOCAL_WORLD_ALIGNED);
    lin_vel_ = world_R_root.transpose() * velocity.linear();
    ang_vel_ = world_R_root.transpose() * velocity.angular();
    lin_acc_ = world_R_root.transpose() * acceleration.linear();
    ang_acc_ = world_R_root.transpose() * acceleration.angular();

    pinocchio::forwardKinematics(model_, *data_, q_, dq_, zero_ddq_);
    const auto bias_acc = pinocchio::getFrameClassicalAcceleration(model_, *data_, ee_frame_idx_, pinocchio::LOCAL_WORLD_ALIGNED);
    bias_acc_.topRows<3>() = world_R_root.transpose() * bias_acc.linear();
    bias_acc_.bottomRows<3>() = world_R_root.transpose() * bias_acc.angular();
}

void ForwardKinematicsPinocchio::set_joints_state(const Ref<const VectorXd>& joints_pos, const Ref<const VectorXd>& joints_vel, const Ref<const VectorXd>& joints_acc)
{
    if (joints_pos.size() != static_cast<int>(joint_bindings_.size()) ||
        joints_vel.size() != static_cast<int>(joint_bindings_.size()) ||
        joints_acc.size() != static_cast<int>(joint_bindings_.size()))
    {
        throw std::runtime_error(log_prefix_ + "::set_joints_state. Joint state size does not match the configured joint list.");
    }

    q_ = pinocchio::neutral(model_);
    dq_.setZero();
    ddq_.setZero();

    for (const auto& binding : joint_bindings_)
    {
        q_(binding.q_index) = joints_pos(binding.input_index);
        dq_(binding.v_index) = joints_vel(binding.input_index);
        ddq_(binding.v_index) = joints_acc(binding.input_index);
    }
}

Eigen::Transform<double, 3, Affine> ForwardKinematicsPinocchio::get_ee_transform()
{
    return transform_;
}

MatrixXd ForwardKinematicsPinocchio::get_jacobian()
{
    return jacobian_;
}

Eigen::Vector3d ForwardKinematicsPinocchio::get_ee_lin_vel()
{
    return lin_vel_;
}

Eigen::Vector3d ForwardKinematicsPinocchio::get_ee_ang_vel()
{
    return ang_vel_;
}

Eigen::Vector3d ForwardKinematicsPinocchio::get_ee_lin_acc()
{
    return lin_acc_;
}

Eigen::Vector3d ForwardKinematicsPinocchio::get_ee_ang_acc()
{
    return ang_acc_;
}

Eigen::VectorXd ForwardKinematicsPinocchio::get_ee_bias_acc()
{
    return bias_acc_;
}

std::vector<std::string> ForwardKinematicsPinocchio::get_joints_list()
{
    return joints_list_;
}
