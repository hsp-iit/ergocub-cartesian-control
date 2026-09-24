/*
 * Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * MIT license. See the accompanying LICENSE file for details.
 */

#ifndef FORWARD_KINEMATICS_PINOCCHIO_H
#define FORWARD_KINEMATICS_PINOCCHIO_H

#include <Eigen/Dense>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ergocub_cartesian_bimanual/ForwardKinematics.h>

#include <memory>
#include <string>
#include <vector>

/**
 * Concrete class implementing the forward kinematics of a kinematic chain
 * using Pinocchio.
 */
class ForwardKinematicsPinocchio : public ForwardKinematics
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ForwardKinematicsPinocchio
        (
            const std::string& urdf_path,
            const std::vector<std::string>& joints_list,
            const std::string& root_frame_name,
            const std::string& ee_frame_name
        );

        ~ForwardKinematicsPinocchio() = default;

        void update() override;

        void set_joints_state(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& joints_vel, const Eigen::Ref<const Eigen::VectorXd>& joints_acc) override;

        Eigen::Transform<double, 3, Eigen::Affine> get_ee_transform() override;

        Eigen::MatrixXd get_jacobian() override;

        Eigen::Vector3d get_ee_lin_vel() override;

        Eigen::Vector3d get_ee_ang_vel() override;

        Eigen::Vector3d get_ee_lin_acc() override;

        Eigen::Vector3d get_ee_ang_acc() override;

        Eigen::VectorXd get_ee_bias_acc() override;

        std::vector<std::string> get_joints_list() override;

    private:
        struct JointBinding
        {
            int input_index{0};
            int q_index{0};
            int v_index{0};
        };

        const std::string log_prefix_ = "ForwardKinematicsPinocchio";

        pinocchio::Model model_;
        std::unique_ptr<pinocchio::Data> data_;

        std::vector<std::string> joints_list_;
        std::vector<JointBinding> joint_bindings_;

        pinocchio::FrameIndex root_frame_idx_{0};
        pinocchio::FrameIndex ee_frame_idx_{0};

        Eigen::VectorXd q_, dq_, ddq_, zero_ddq_;
        Eigen::MatrixXd full_jacobian_, jacobian_;
        Eigen::Transform<double, 3, Eigen::Affine> transform_;
        Eigen::Vector3d lin_vel_, ang_vel_, lin_acc_, ang_acc_;
        Eigen::VectorXd bias_acc_;
};

#endif /* FORWARD_KINEMATICS_PINOCCHIO_H */
