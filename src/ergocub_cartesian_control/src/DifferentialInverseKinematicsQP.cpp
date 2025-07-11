// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef DIFFERENTIAL_INVERSE_KINEMATICS_QP_CPP
#define DIFFERENTIAL_INVERSE_KINEMATICS_QP_CPP

#include <DifferentialInverseKinematicsQP.h>

#include <yarp/os/LogStream.h>

using namespace proxsuite::proxqp;

DifferentialInverseKinematicsQP::DifferentialInverseKinematicsQP(
    double sampling_time,
    double limits_param,
    const Eigen::VectorXd &joint_acc_weight,
    const Eigen::VectorXd &position_param,
    const Eigen::MatrixXd &orientation_param,
    const Eigen::VectorXd &joint_pos_param,
    const Eigen::MatrixXd &joint_ref,
    bool verbose) : sampling_time_(sampling_time),
                    limits_param_(limits_param),
                    joint_acc_weight_(joint_acc_weight),
                    position_param_(position_param),
                    orientation_param_(orientation_param),
                    joint_pos_param_(joint_pos_param),
                    joint_ref_(joint_ref),
                    bias_acc_(Eigen::VectorXd::Zero(6)),
                    verbose_(verbose)
{
    std::string error_message = class_name_ + "::ctor(). Error: parameter(s) not valid:";
    bool error = false;

    if (limits_param_ < 0)
    {
        error = true;
        error_message += " limits_param";
    }
    if (joint_acc_weight_(0) < 0)
    {
        error = true;
        error_message += " joint_acc_weight";
    }
    if (position_param_(0) < 0 || position_param_(1) < 0 || position_param_(2) < 0)
    {
        error = true;
        error_message += " position_param";
    }
    if (orientation_param_(0) < 0 || orientation_param_(1) < 0 || orientation_param_(2) < 0)
    {
        error = true;
        error_message += " orientation_param";
    }
    if (joint_pos_param_(0) < 0 || joint_pos_param_(1) < 0 || joint_pos_param_(2) < 0)
    {
        error = true;
        error_message += " joint_pos_param";
    }

    if (error)
    {
        error_message += ".";
        throw std::runtime_error(error_message);
    }

    max_manip_ = 0.0;
    manip_ = 0.0;
    weight_manip_function_ = 0.0;
}

std::optional<Eigen::VectorXd> DifferentialInverseKinematicsQP::eval_reference_velocities()
{
    /* Jacobians. */
    Eigen::MatrixXd J_pos = jacobian_.topRows<3>();
    Eigen::MatrixXd J_rot = jacobian_.bottomRows<3>();

    /* Position error. */
    Eigen::Vector3d e_p = desired_transform_.translation() - transform_.translation();

    /* Linear velocity error. */
    Eigen::Vector3d e_v = desired_lin_vel_ - J_pos * joints_vel_;

    /* Orientation error. */
    Eigen::AngleAxisd e_o_aa(desired_transform_.rotation() * transform_.rotation().transpose());
    Eigen::Vector3d e_o = e_o_aa.axis() * e_o_aa.angle();

    /* Angular velocity error. */
    Eigen::Vector3d e_w = desired_ang_vel_ - J_rot * joints_vel_;

    /* Reference values. */
    Eigen::Vector3d lin_ref = - bias_acc_.topRows<3>() + position_param_(1) * e_p + position_param_(2) * e_v;
    Eigen::Vector3d ang_ref = - bias_acc_.bottomRows<3>() + orientation_param_(1) * e_o + orientation_param_(2) * e_w;

    /*
    * Setup the QP-related matrices. In this formulation,
    * position control has greater priority than orientation control.
    */


    /*
    * min || REF - J x||^2_W = min x^T J^T W J x - 2 x^T J^T W REF
    *  x                        x
    */

    Eigen::MatrixXd P = J_rot.transpose() * orientation_param_(0) * J_rot;
    Eigen::MatrixXd q = - 2.0 * J_rot.transpose() * orientation_param_(0) * ang_ref;

    Eigen::MatrixXd A;
    Eigen::MatrixXd b;

    if (position_param_(0) > 0.0)
    { /*
          Write the QP such to have a formulation that is equivalent to introducing slack variables.
          In this formulation, position control has no longer greater priority than orientation control,
          but a priority can still be given by properly selecting the weights.
      */

        P += J_pos.transpose() * position_param_(0) * J_pos;
        q += - 2.0 * J_pos.transpose() * position_param_(0) * lin_ref;

        manip_ = sqrt((jacobian_ * jacobian_.transpose()).determinant());

        if (max_manip_ < manip_)
            max_manip_ = manip_;

        weight_manip_function_ = pow(1 - (manip_ / max_manip_), 2);

    }
    else
    { /* Slack variables are not introduced into the QP, position control is prioritized. */
        A = J_pos;
        b = lin_ref;
    }

    /* Introduce joint accelerations term into cost function. */
    if (joint_acc_weight_(0) > 0.0)
    {
        P += Eigen::MatrixXd::Identity(P.rows(), P.cols()) * joint_acc_weight_(0) * weight_manip_function_;
    }

    /* Introduce joint position control term into cost function. */
    if (joint_pos_param_(0) > 0.0)
    {

        /*
        * min || x - ref ||^2_W = min x^T W x - 2 x^T W ref
        *  x                       x
        */

        P += Eigen::MatrixXd::Identity(joints_.size(), joints_.size()) * joint_pos_param_(0);

        const Eigen::MatrixXd des_joints_vel = Eigen::MatrixXd::Zero(joints_.size(), 1);
        const Eigen::MatrixXd des_joints_acc = Eigen::MatrixXd::Zero(joints_.size(), 1);

        Eigen::MatrixXd ddq_ref(joints_.size(), 1);

        Eigen::MatrixXd JP_1 = joint_pos_param_(1) * Eigen::MatrixXd::Identity(joints_.size(), joints_.size());
        Eigen::MatrixXd JP_2 = joint_pos_param_(2) * Eigen::MatrixXd::Identity(joints_.size(), joints_.size());

        // Increment torso roll PD gains in order to prefer other joints motion.
        JP_1(0,0)*=5;
        JP_2(0,0)*=5;
        JP_1(1,1)*=5;
        JP_2(1,1)*=5;
        JP_1(2,2)*=5;
        JP_2(2,2)*=5;
        ddq_ref = des_joints_acc + JP_1 * (joint_ref_ - joints_) + JP_2 * (des_joints_vel - joints_vel_);

        ddq_ref *= - 2.0 * joint_pos_param_(0);
        q += ddq_ref;
    }

    /* Add limiter contribution to the constraint.*/
    Eigen::MatrixXd G_limiter;
    Eigen::VectorXd h_limiter;
    std::tie(G_limiter, h_limiter) = limit_constraint(joints_, joints_lower_limits_, joints_upper_limits_, joints_limits_gains_);

    /* Try to solve the problem. */
    if (position_param_(0) > 0.0)
        reference_joints_velocities_ = solve(P, q, G_limiter, h_limiter);
    else
        reference_joints_velocities_ = solve(P, q, A, b, G_limiter, h_limiter);

    return reference_joints_velocities_;
}

void DifferentialInverseKinematicsQP::set_robot_state(
        const Eigen::Ref<const Eigen::VectorXd> &joints,
        const Eigen::Ref<const Eigen::VectorXd> &joints_vel,
        const Eigen::Transform<double, 3, Eigen::Affine> &transform,
        const Eigen::Ref<const Eigen::MatrixXd> &jacobian,
        const Eigen::Ref<const Eigen::VectorXd> &bias_acc)
{
    joints_ = joints;
    joints_vel_ = joints_vel;
    transform_ = transform;
    jacobian_ = jacobian;
    bias_acc_ = bias_acc;
}

void DifferentialInverseKinematicsQP::set_desired_ee_transform(const Eigen::Transform<double, 3, Eigen::Affine> &transform)
{
    desired_transform_ = transform;
}


void DifferentialInverseKinematicsQP::set_desired_ee_twist(const Eigen::Ref<const Eigen::Vector3d> &lin_vel, const Eigen::Ref<const Eigen::Vector3d> &ang_vel)
{
    desired_lin_vel_ = lin_vel;
    desired_ang_vel_ = ang_vel;
}

void DifferentialInverseKinematicsQP::set_joint_limits(
    const Eigen::Ref<const Eigen::VectorXd> &lower_limits,
    const Eigen::Ref<const Eigen::VectorXd> &upper_limits,
    const Eigen::Ref<const Eigen::VectorXd> &gains)
{
    joints_lower_limits_ = lower_limits;
    joints_upper_limits_ = upper_limits;
    joints_limits_gains_ = gains;
}

std::optional<Eigen::VectorXd> DifferentialInverseKinematicsQP::solve(
    const Eigen::Ref<const Eigen::MatrixXd> &P,
    const Eigen::Ref<const Eigen::VectorXd> &q,
    const Eigen::Ref<const Eigen::MatrixXd> &A,
    const Eigen::Ref<const Eigen::VectorXd> &b,
    const Eigen::Ref<const Eigen::MatrixXd> &G,
    const Eigen::Ref<const Eigen::VectorXd> &h)
{
    /* Solve QP with prioritized position control. */
    dense::QP<double> solver(P.rows(), A.rows(), G.rows());
    solver.init(P, q, A, b, G, proxsuite::nullopt, h);

    if (old_solution_.x.size() == 0)
        solver.solve();
    else
        solver.solve(old_solution_.x, old_solution_.y, old_solution_.z);

    if (solver.results.info.status == QPSolverOutput::PROXQP_SOLVED)
    {
        old_solution_ = solver.results;
        return solver.results.x;
    }

    return {};
}

std::optional<Eigen::VectorXd> DifferentialInverseKinematicsQP::solve(
    const Eigen::Ref<const Eigen::MatrixXd> &P,
    const Eigen::Ref<const Eigen::VectorXd> &q,
    const Eigen::Ref<const Eigen::MatrixXd> &G,
    const Eigen::Ref<const Eigen::VectorXd> &h)
{
    dense::QP<double> solver(P.rows(), 0, G.rows());
    solver.init(P, q, proxsuite::nullopt, proxsuite::nullopt, G, proxsuite::nullopt, h);

    if (old_solution_.x.size() == 0)
        solver.solve();
    else
        solver.solve(old_solution_.x, old_solution_.y, old_solution_.z);

    if (solver.results.info.status == QPSolverOutput::PROXQP_SOLVED)
    {
        old_solution_ = solver.results;
        return solver.results.x;
    }

    return {};
}

/**
 * Implement limits as described in:
 * Kanoun, Oussama. "Real-time prioritized kinematic control under inequality constraints for redundant manipulators." Robotics: Science and Systems. Vol. 7. 2012.
 *
 * Namely, they are implemented as:
 * \f[
 * G_l \dot{q} \preccurlyeq h_l
 * \f]
 * with
 * \f[
 * G_l =
 * \begin{bmatrix}
 *     I_{n}\\
 *     -I_{n}
 * \end{bmatrix}
 * \f]
 * and
 * \f[
 * h_l =
 * \begin{bmatrix}
 *     \mathrm{diag}(K_l)(q_{U} - q)\\
 *    -\mathrm{diag}(K_l)(q_{L} - q)
 * \end{bmatrix}
 * \f]
 * with \f$q \in \mathbb{R}^{n}\f$ the current joints.
 *
 *@param joints The current joint configuration \f$q\f$.
 *@param joints_lower_limit The joint lower limits \f$q_{L}\f$.
 *@param joints_upper_limit The joint upper limits \f$q_{U}\f$.
 *@param gains The vector of gains \f$K_l\f$.
 *@return The tuple \f$(G_l, b_l)\f$.
 */
std::tuple<Eigen::MatrixXd, Eigen::VectorXd> DifferentialInverseKinematicsQP::limit_constraint(const Eigen::Ref<const Eigen::VectorXd> &joints, const Eigen::Ref<const Eigen::VectorXd> &joints_lower_limit, const Eigen::Ref<const Eigen::VectorXd> &joints_upper_limit, const Eigen::Ref<const Eigen::VectorXd> &gains)
{
    /* Sanity check. */
    if (verbose_)
    {
        for (std::size_t i = 0; i < joints.size(); i++)
            if ((joints(i) > joints_upper_limit(i)) || (joints(i) < joints_lower_limit(i)))
            {
                std::string error_message = "Limiter::limit_constraint<LimiterType::DistanceBased>(). ";
                error_message += "Warning: limits have been reached or violated for joint ";
                error_message += std::to_string(i);
                error_message += " ";
                error_message += "(";
                error_message += std::to_string(joints_lower_limit(i));
                error_message += " !<= ";
                error_message += std::to_string(joints(i));
                error_message += " !<= ";
                error_message += std::to_string(joints_upper_limit(i));
                error_message += ")";

                yWarning() << error_message;
            }
    }

    /*
    * ddq^ = optimal acceleration.
    * dq^ = ddq^ * dt + dq;
    *  q^ =  dq^ * dt +  q;
    *
    * q^ <= qu -> (ddq^ * dt + dq) * dt + q <= qu ->   ddq^ <=   ((qu - q)/dt - dq)/dt
    * ql <= q^ -> ql <= (ddq^ * dt + dq) * dt + q -> - ddq^ <= - ((ql - q)/dt - dq)/dt
    *
    * Kl : 0 < Kl <= 1 ->     ddq^ <=   Kl * ((qu - q)/dt - dq)/dt
    *                       - ddq^ <= - Kl * ((ql - q)/dt - dq)/dt
    */

    Eigen::MatrixXd G_l(joints.size() * 2, joints.size());
    Eigen::VectorXd h_l(joints.size() * 2);

    G_l.topRows(joints.size()) = Eigen::MatrixXd::Identity(joints.size(), joints.size());
    G_l.bottomRows(joints.size()) = -Eigen::MatrixXd::Identity(joints.size(), joints.size());

    h_l.head(joints.size()) = gains.asDiagonal() * ((joints_upper_limit - joints)/sampling_time_ - joints_vel_)/sampling_time_;
    h_l.tail(joints.size()) = -1.0 * gains.asDiagonal() * ((joints_lower_limit - joints)/sampling_time_ - joints_vel_)/sampling_time_;

    return std::make_tuple(G_l, h_l);
}

void addBlockOnDiag(Eigen::MatrixXd &M, const Eigen::MatrixXd &M_add)
{
    if (M.cols() == 0)
    {
        M = M_add;
    }
    else
    {
        int M_col = M.cols();
        int M_row = M.rows();

        M.conservativeResize(M_row + M_add.rows(), M_col + M_add.cols());

        M.topRightCorner(M_row, M_add.cols()).setZero();
        M.bottomLeftCorner(M_add.rows(), M_col).setZero();
        M.bottomRightCorner(M_add.rows(), M_add.cols()) = M_add;
    }
}

#endif /* DIFFERENTIAL_INVERSE_KINEMATICS_QP_CPP */
