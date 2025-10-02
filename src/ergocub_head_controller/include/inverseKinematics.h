#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <Eigen/Dense>

#include <proxsuite/proxqp/dense/dense.hpp>

#include <optional>
#include <unordered_map>

#include <inverseKinematicsParameters.h>


class InverseKinematics
{
public:
    void configure(const std::size_t& number_joints, const InverseKinematicsParameters& ik_parameters, const double& sample_time);

    /**
     * @param q_a Eigen::VectorXd containing actuated joints.
     * @param q_na Eigen::VectorXd containing non actuated joints.
     * @param qd_na Eigen::VectorXd containing non actuated joints velocities.
     * @param J_a Eigen::MatrixXd containing the Jacobian associated to actuated joints.
     * @param J_na Eigen::MatrixXd containing the Jacobian associated to non actuated joints.
     * @param T Eigen::Transform<double, 3, Eigen::Affine> containing the pose of the gazer, i.e., of the camera.
     */
    void setGazerState(const Eigen::VectorXd& q_a, const Eigen::VectorXd& q_na, const Eigen::VectorXd& qd_na, const Eigen::MatrixXd& J_a, const Eigen::MatrixXd& J_na, const Eigen::Transform<double, 3, Eigen::Affine>& T);

    /**
     * @param R Eigen::Matrix3d containing the current desired gazing orientation.
     */
    void setDesiredGazingOrientation(const Eigen::Matrix3d& R);

    void setLimits(const Eigen::VectorXd& lower_limits, const Eigen::VectorXd& upper_limits);

    /**
     * Solves the optimization problem in order to find the reference joints velocities.
     */
    void update();

    std::optional<Eigen::VectorXd> getReferenceJointsVelocity();

private:
    /* Actuated and non actuated joints. */
    Eigen::VectorXd q_a_;
    Eigen::VectorXd q_na_;

    /* Lower and upper limits for the actuated joints. */
    Eigen::VectorXd q_a_lim_u_;
    Eigen::VectorXd q_a_lim_l_;

    /* Non actuated joints velocities. */
    Eigen::VectorXd qd_na_;

    /* Jacobian of the actuated and non actuated joints. */
    Eigen::MatrixXd J_a_;
    Eigen::MatrixXd J_na_;

    /* Pose of the gazer, i.e., of the camera. */
    Eigen::Transform<double, 3, Eigen::Affine> T_;

    /* Desired head orientation and velocity. */
    Eigen::Matrix3d R_des_;

    /* Solver. */
    std::unique_ptr<proxsuite::proxqp::dense::QP<double>> solver_;
    std::optional<Eigen::VectorXd> solution_;

    /* IK parameters. */
    InverseKinematicsParameters ik_parameters_;

    /* Sample time. */
    double sample_time_;
};

#endif /* INVERSE_KINEMATIS_H */