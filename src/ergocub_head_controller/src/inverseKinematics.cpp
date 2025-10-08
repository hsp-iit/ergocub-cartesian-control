#include <inverseKinematics.h>

#include <iostream>


void InverseKinematics::configure(const std::size_t& number_joints, const InverseKinematicsParameters& ik_parameters, const double& sample_time)
{
    /* The size of the problem is equal to the number of joints. */
    std::size_t size = number_joints;

    std::size_t number_eq_constraint = 0;

    /* Joints limits are implemented as inequality constraints on the joints velocities, for both lower and upper limits,
       hence the number of inequality constraints is equal to twice the number of joints. */
    std::size_t number_ineq_constraint = 2 * number_joints;

    /* Initialize the solver. */
    solver_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(size, number_eq_constraint, number_ineq_constraint);

    /* Initialize the sample time. */
    sample_time_ = sample_time;

    /* Copy ik parameters. */
    ik_parameters_ = ik_parameters;

}


void InverseKinematics::setGazerState(const Eigen::VectorXd& q_a, const Eigen::VectorXd& q_na, const Eigen::VectorXd& qd_na, const Eigen::MatrixXd& J_a, const Eigen::MatrixXd& J_na, const Eigen::Transform<double, 3, Eigen::Affine>& T)
{
    q_a_ = q_a;
    q_na_ = q_na;
    qd_na_ = qd_na;

    J_a_ = J_a;
    J_na_ = J_na;

    T_ = T;
}


void InverseKinematics::setDesiredGazingOrientation(const Eigen::Matrix3d& R)
{
    R_des_ = R;
}


void InverseKinematics::setLimits(const Eigen::VectorXd& lower_limits, const Eigen::VectorXd& upper_limits)
{
    q_a_lim_u_ = upper_limits;
    q_a_lim_l_ = lower_limits;
}


void InverseKinematics::update()
{

    /* Orientation error. */
    Eigen::AngleAxisd e_o_aa(R_des_* T_.rotation().transpose());
    Eigen::Vector3d orientation_error = e_o_aa.axis() * e_o_aa.angle();
    Eigen::Vector3d orientation_error_term = ik_parameters_.p_gain() * orientation_error;

    /* Rejection term due to motion of non actuated joints. */
    Eigen::Vector3d rejection_term = Eigen::Vector3d::Zero();
    if (q_na_.size() > 0)
    {
        rejection_term = - J_na_.bottomRows(3) * qd_na_;
    }

    /* QP-based inverse kinematics. */

    /** Cost term - joints velocity minimization. */
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(q_a_.size(), q_a_.size()) * ik_parameters_.weight_vel();
    Eigen::VectorXd g = Eigen::VectorXd::Zero(q_a_.size());

    /** Cost term - gaze tracking task: Ja qd_a = orientation_error + rejection_error. */;
    H += J_a_.bottomRows(3).transpose() * J_a_.bottomRows(3);
    g += -2 * J_a_.bottomRows(3).transpose() * (orientation_error_term + rejection_term);

    /* Inequality constraints - joints limits. */
    Eigen::MatrixXd C(q_a_.size() * 2, q_a_.size());
    Eigen::VectorXd u(q_a_.size() * 2);

    C.topRows(q_a_.size()) = Eigen::MatrixXd::Identity(q_a_.size(), q_a_.size());
    C.bottomRows(q_a_.size()) = -Eigen::MatrixXd::Identity(q_a_.size(), q_a_.size());

    u.head(q_a_.size()) = ik_parameters_.gain_limits() * (q_a_lim_u_ - q_a_) / sample_time_;
    u.tail(q_a_.size()) = -1.0 * ik_parameters_.gain_limits() * (q_a_lim_l_ - q_a_) / sample_time_;

    /* Initialize the solver. */
    solver_->init(H, g, proxsuite::nullopt, proxsuite::nullopt, C, proxsuite::nullopt, u);

    /* Solve. */
    solver_->solve();

    /* Check solution validity. */
    if (solver_->results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED)
        solution_ = solver_->results.x;
    else
        solution_.reset();
}


std::optional<Eigen::VectorXd> InverseKinematics::getReferenceJointsVelocity()
{
    return solution_;
}
