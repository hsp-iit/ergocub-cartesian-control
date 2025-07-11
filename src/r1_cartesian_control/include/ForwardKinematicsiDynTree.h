// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FORWARD_KINEMATICS_IDYNTREE_H
#define FORWARD_KINEMATICS_IDYNTREE_H

#include <Eigen/Dense>

#include <iDynTree/KinDynComputations.h>

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

        void set_joints_state(const Eigen::Ref<const Eigen::VectorXd>& joints) override;

        Eigen::Transform<double, 3, Eigen::Affine> get_ee_transform() override;

        Eigen::MatrixXd get_jacobian() override;

    private:
        const std::string log_prefix_ = "ForwardKinematicsiDynTree";

        iDynTree::KinDynComputations chain_;

        iDynTree::FrameIndex root_frame_idx_;

        iDynTree::FrameIndex ee_frame_idx_;

        Eigen::Transform<double, 3, Eigen::Affine> transform_;

        Eigen::MatrixXd jacobian_;
    };


#endif /* FORWARD_KINEMATICS_IDYNTREE_H */
