#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <Eigen/Dense>

#include <iDynTree/KinDynComputations.h>

#include <string>
#include <vector>


class ForwardKinematics
{
public:

    bool configure(const std::string& urdf_path, const std::vector<std::string>& joints_list, const std::vector<std::string>& actuated_joints_list, const std::string& root_frame_name, const std::string& ee_frame_name);

    void update();

    bool setJoints(const Eigen::VectorXd& joints_non_actuated, const Eigen::VectorXd& joints_actuated);

    Eigen::Transform<double, 3, Eigen::Affine> getTransform();

    Eigen::MatrixXd getJacobian();

    Eigen::MatrixXd getJacobianActuatedJoints();

    Eigen::MatrixXd getJacobianUnactuatedJoints();

private:
    /* iDynTree-related. */
    iDynTree::KinDynComputations chain_;
    iDynTree::FrameIndex root_frame_idx_;
    iDynTree::FrameIndex ee_frame_idx_;

    /* Current transform. */
    Eigen::Transform<double, 3, Eigen::Affine> transform_;

    /* Current Jacobian. */
    Eigen::MatrixXd jacobian_;

    /* List of actuated and non actuated joints indexes. */
    std::vector<int> non_actuated_joints_indexes_;
    std::vector<int> actuated_joints_indexes_;

    /* Class name. */
    const std::string class_name_ = "ForwardKinematics";
};


#endif /* FORWARD_KINEMATICS_H */
