// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FORWARD_KINEMATICS_IDYNTREE_H
#define FORWARD_KINEMATICS_IDYNTREE_H

#include <Eigen/Dense>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/EigenHelpers.h>

#include <ForwardKinematics.h>

#include <string>
#include <vector>

/**
 * Concrete class implementing the forward kinematics of a kinematic chain
 * using the iDynTree library given the path to a URDF file, the list of joints of interest,
 * and the names of the root and end-effector frames.
 */
class ForwardKinematicsiDynTree : public ForwardKinematics
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * Constructor.
         * @param urdf_path Path to the URDF file.
         * @param joints_list A list of strings containing the names of the joints of interest.
         * @param root_frame_name The name of root frame.
         * @param ee_frame_name The name of the frame attached to the end-effector.
         */
        ForwardKinematicsiDynTree
        (
            const std::string& urdf_path,
            const std::vector<std::string>& joints_list,
            const std::string& root_frame_name,
            const std::string& ee_frame_name
        );

        ~ForwardKinematicsiDynTree() = default;

        void update() override;

        void set_joints_state(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& joints_vel, const Eigen::Ref<const Eigen::VectorXd>& joints_acc) override;

        Eigen::Transform<double, 3, Eigen::Affine> get_ee_transform() override;

        Eigen::MatrixXd get_jacobian() override;

        Eigen::Vector3d get_ee_lin_vel() override;

        Eigen::Vector3d get_ee_ang_vel() override;

        Eigen::Vector3d get_ee_lin_acc() override;

        Eigen::Vector3d get_ee_ang_acc() override;

        Eigen::VectorXd get_ee_bias_acc() override;

    private:
        const std::string log_prefix_ = "ForwardKinematicsiDynTree";

        iDynTree::KinDynComputations chain_;

        iDynTree::FrameIndex root_frame_idx_;

        iDynTree::FrameIndex ee_frame_idx_;

        Eigen::Transform<double, 3, Eigen::Affine> transform_;

        Eigen::MatrixXd jacobian_;

        Eigen::Vector3d lin_vel_, ang_vel_;

        Eigen::Vector3d lin_acc_, ang_acc_;

        Eigen::VectorXd joint_acc_, bias_acc_;
    };


#endif /* FORWARD_KINEMATICS_IDYNTREE_H */
