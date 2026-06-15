/*
 * Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * MIT license. See the accompanying LICENSE file for details.
 */

#ifndef DIFFERENTIAL_INVERSE_KINEMATICS_QP_CPP
#define DIFFERENTIAL_INVERSE_KINEMATICS_QP_CPP

#include <ergocub_cartesian_bimanual/DifferentialInverseKinematicsQP.h>

#include <yarp/os/LogStream.h>

#include <Eigen/Dense>

using namespace proxsuite::proxqp;

DifferentialInverseKinematicsQP::DifferentialInverseKinematicsQP(
    const double sampling_time,
    const bool verbose,
    const int right_arm_joints,
    const int left_arm_joints,
    const int torso_joints,
    const Eigen::VectorXd &joint_pos_weights,
    const Eigen::VectorXd &joint_pos_p_gain,
    const Eigen::VectorXd &joint_pos_d_gain,
    const Eigen::VectorXd &cartesian_pos_weight,
    const Eigen::VectorXd &cartesian_pos_p_gain,
    const Eigen::VectorXd &cartesian_pos_d_gain,
    const Eigen::VectorXd &cartesian_ori_weight,
    const Eigen::VectorXd &cartesian_ori_p_gain,
    const Eigen::VectorXd &cartesian_ori_d_gain,
    const double improve_manip_dyn,
    const double improve_manip_th,
    const Eigen::VectorXd &joint_ref):
    sampling_time_(sampling_time),
    verbose_(verbose),
    right_arm_joints_(right_arm_joints),
    left_arm_joints_(left_arm_joints),
    torso_joints_(torso_joints),
    joint_pos_weights_(joint_pos_weights),
    joint_pos_p_gain_(joint_pos_p_gain),
    joint_pos_d_gain_(joint_pos_d_gain),
    cartesian_pos_weight_(cartesian_pos_weight),
    cartesian_pos_p_gain_(cartesian_pos_p_gain),
    cartesian_pos_d_gain_(cartesian_pos_d_gain),
    cartesian_ori_weight_(cartesian_ori_weight),
    cartesian_ori_p_gain_(cartesian_ori_p_gain),
    cartesian_ori_d_gain_(cartesian_ori_d_gain),
    improve_manip_dyn_(improve_manip_dyn),
    improve_manip_th_(improve_manip_th),
    joint_ref_(joint_ref),
    right_bias_acc_(Eigen::VectorXd::Zero(6)),
    left_bias_acc_(Eigen::VectorXd::Zero(6)),
    right_transform_(Eigen::Transform<double, 3, Eigen::Affine>::Identity()),
    left_transform_(Eigen::Transform<double, 3, Eigen::Affine>::Identity()),
    right_desired_transform_(Eigen::Transform<double, 3, Eigen::Affine>::Identity()),
    right_lin_vel_(Eigen::Vector3d::Zero()),
    right_ang_vel_(Eigen::Vector3d::Zero()),
    right_lin_acc_(Eigen::Vector3d::Zero()),
    right_ang_acc_(Eigen::Vector3d::Zero()),
    right_desired_lin_vel_(Eigen::Vector3d::Zero()),
    right_desired_ang_vel_(Eigen::Vector3d::Zero()),
    right_desired_lin_acc_(Eigen::Vector3d::Zero()),
    right_desired_ang_acc_(Eigen::Vector3d::Zero()),
    left_desired_transform_(Eigen::Transform<double, 3, Eigen::Affine>::Identity()),
    left_lin_vel_(Eigen::Vector3d::Zero()),
    left_ang_vel_(Eigen::Vector3d::Zero()),
    left_lin_acc_(Eigen::Vector3d::Zero()),
    left_ang_acc_(Eigen::Vector3d::Zero()),
    left_desired_lin_vel_(Eigen::Vector3d::Zero()),
    left_desired_ang_vel_(Eigen::Vector3d::Zero()),
    left_desired_lin_acc_(Eigen::Vector3d::Zero()),
    left_desired_ang_acc_(Eigen::Vector3d::Zero())
{
    std::string error_message = "[" + class_name_ + "::ctor]. Error: parameter(s) not valid:";
    bool error = false;

    if (sampling_time_ <= 0)
    {
        error = true;
        error_message += " sampling_time";
    }

    int active_joints_num = right_arm_joints + left_arm_joints + torso_joints;

    auto hasNonPositive = [](const Eigen::VectorXd& v) {
        for (int i = 0; i < v.size(); ++i) if (v(i) <= 0.0) return true;
        return false;
    };
    auto hasNegative = [](const Eigen::VectorXd& v) {
        for (int i = 0; i < v.size(); ++i) if (v(i) < 0.0) return true;
        return false;
    };

    if (hasNonPositive(joint_pos_weights) || (joint_pos_weights.size() != active_joints_num))
    {
        error = true;
        error_message += " joint_pos_weights";
    }

    if (hasNonPositive(joint_pos_p_gain) || (joint_pos_p_gain.size() != active_joints_num))
    {
        error = true;
        error_message += " joint_pos_p_gain";
    }

    if (hasNonPositive(joint_pos_d_gain) || (joint_pos_d_gain.size() != active_joints_num))
    {
        error = true;
        error_message += " joint_pos_d_gain";
    }

    // Pesi/gain cartesiani: CONSENTI 0 per EE disattive (solo vietati i negativi).
    if (cartesian_pos_weight.size() != 2 || hasNegative(cartesian_pos_weight))
    {
        error = true;
        error_message += " cartesian_pos_weight";
    }
    if (cartesian_pos_p_gain.size() != 2 || hasNegative(cartesian_pos_p_gain))
    {
        error = true;
        error_message += " cartesian_pos_p_gain";
    }
    if (cartesian_pos_d_gain.size() != 2 || hasNegative(cartesian_pos_d_gain))
    {
        error = true;
        error_message += " cartesian_pos_d_gain";
    }
    if (cartesian_ori_weight.size() != 2 || hasNegative(cartesian_ori_weight))
    {
        error = true;
        error_message += " cartesian_ori_weight";
    }
    if (cartesian_ori_p_gain.size() != 2 || hasNegative(cartesian_ori_p_gain))
    {
        error = true;
        error_message += " cartesian_ori_p_gain";
    }
    if (cartesian_ori_d_gain.size() != 2 || hasNegative(cartesian_ori_d_gain))
    {
        error = true;
        error_message += " cartesian_ori_d_gain";
    }

    if (joint_ref.size() != active_joints_num)
    {
        error = true;
        error_message += " joint_ref.size()";
    }

    if (error)
    {
        error_message += ".";
        throw std::runtime_error(error_message);
    }

    joints_ = Eigen::VectorXd::Zero(active_joints_num);
    joints_vel_ = Eigen::VectorXd::Zero(active_joints_num);
    joints_lower_limits_ = Eigen::VectorXd::Zero(active_joints_num);
    joints_upper_limits_ = Eigen::VectorXd::Zero(active_joints_num);
    right_jacobian_ = Eigen::MatrixXd::Zero(6, active_joints_num);
    left_jacobian_ = Eigen::MatrixXd::Zero(6, active_joints_num);
    qp_results_ = Eigen::VectorXd::Zero(active_joints_num);
    J_pos_ = Eigen::MatrixXd::Zero(6, active_joints_num);
    J_ori_ = Eigen::MatrixXd::Zero(6, active_joints_num);

    W_lin_cart_ = Eigen::DiagonalMatrix<double, 6> (cartesian_pos_weight_(0), cartesian_pos_weight_(0), cartesian_pos_weight_(0),
                                                    cartesian_pos_weight_(1), cartesian_pos_weight_(1), cartesian_pos_weight_(1));

    W_ang_cart_ = Eigen::DiagonalMatrix<double, 6> (cartesian_ori_weight_(0), cartesian_ori_weight_(0), cartesian_ori_weight_(0),
                                                    cartesian_ori_weight_(1), cartesian_ori_weight_(1), cartesian_ori_weight_(1));

    lin_Kp_ = Eigen::DiagonalMatrix<double, 6> (cartesian_pos_p_gain_(0),cartesian_pos_p_gain_(0),cartesian_pos_p_gain_(0),
                                                cartesian_pos_p_gain_(1),cartesian_pos_p_gain_(1),cartesian_pos_p_gain_(1));

    lin_Kd_ = Eigen::DiagonalMatrix<double, 6> (cartesian_pos_d_gain_(0),cartesian_pos_d_gain_(0),cartesian_pos_d_gain_(0),
                                                cartesian_pos_d_gain_(1),cartesian_pos_d_gain_(1),cartesian_pos_d_gain_(1));

    ang_Kp_ = Eigen::DiagonalMatrix<double, 6> (cartesian_ori_p_gain_(0),cartesian_ori_p_gain_(0),cartesian_ori_p_gain_(0),
                                                cartesian_ori_p_gain_(1),cartesian_ori_p_gain_(1),cartesian_ori_p_gain_(1));

    ang_Kd_ = Eigen::DiagonalMatrix<double, 6> (cartesian_ori_d_gain_(0),cartesian_ori_d_gain_(0),cartesian_ori_d_gain_(0),
                                                cartesian_ori_d_gain_(1),cartesian_ori_d_gain_(1),cartesian_ori_d_gain_(1));

}

bool DifferentialInverseKinematicsQP::set_joint_limits(
    const Eigen::Ref<const Eigen::VectorXd> &lower_limits,
    const Eigen::Ref<const Eigen::VectorXd> &upper_limits,
    const Eigen::Ref<const Eigen::VectorXd> &gains)
{
    // joints are in the form [right_arm left_arm torso]

    joints_lower_limits_ = lower_limits;
    joints_upper_limits_ = upper_limits;

    auto gain_at = [&](int i, double def) -> double {
        return (i < gains.size()) ? gains(i) : def;
    };

    // Validate only provided gains
    for (int i = 0; i < gains.size(); i++)
    {
        if (gains(i) < 0.0 || gains(i) > 1.0)
        {
            yError() << "[" << class_name_ << "::set_joint_limits] Gain outside [0,1] at index " << i;
            return false;
        }
    }

    // Defaults to 1.0 if not provided
    double gR = gain_at(0, 1.0);
    double gL = gain_at(1, 1.0);
    double gT = gain_at(2, 1.0);

    joints_limits_gains_.resize(right_arm_joints_ + left_arm_joints_ + torso_joints_);

    for (int i = 0; i < right_arm_joints_; i++)
        joints_limits_gains_(i) = gR;

    for (int i = right_arm_joints_; i < right_arm_joints_ + left_arm_joints_; i++)
        joints_limits_gains_(i) = gL;

    for (int i = right_arm_joints_ + left_arm_joints_; i < right_arm_joints_ + left_arm_joints_ + torso_joints_; i++)
        joints_limits_gains_(i) = gT;

    return true;
}


void DifferentialInverseKinematicsQP::set_robot_state(
        const Eigen::Ref<const Eigen::VectorXd> &joints,
        const Eigen::Ref<const Eigen::VectorXd> &joints_vel,
        const Eigen::Transform<double, 3, Eigen::Affine> &r_transform,
        const Eigen::Ref<const Eigen::VectorXd> &r_lin_vel,
        const Eigen::Ref<const Eigen::VectorXd> &r_ang_vel,
        const Eigen::Ref<const Eigen::VectorXd> &r_lin_acc,
        const Eigen::Ref<const Eigen::VectorXd> &r_ang_acc,
        const Eigen::Ref<const Eigen::MatrixXd> &r_jacobian,
        const Eigen::Ref<const Eigen::VectorXd> &r_bias_acc,
        const Eigen::Transform<double, 3, Eigen::Affine> &l_transform,
        const Eigen::Ref<const Eigen::VectorXd> &l_lin_vel,
        const Eigen::Ref<const Eigen::VectorXd> &l_ang_vel,
        const Eigen::Ref<const Eigen::VectorXd> &l_lin_acc,
        const Eigen::Ref<const Eigen::VectorXd> &l_ang_acc,
        const Eigen::Ref<const Eigen::MatrixXd> &l_jacobian,
        const Eigen::Ref<const Eigen::VectorXd> &l_bias_acc)
{
    // joints are in the form [right_arm left_arm torso]
    joints_ = joints;
    joints_vel_ = joints_vel;

    right_transform_ = r_transform;
    right_lin_vel_ = r_lin_vel;
    right_ang_vel_ = r_ang_vel;
    right_lin_acc_ = r_lin_acc;
    right_ang_acc_ = r_ang_acc;
    right_jacobian_ = r_jacobian;
    right_bias_acc_ = r_bias_acc;

    left_transform_ = l_transform;
    left_lin_vel_ = l_lin_vel;
    left_ang_vel_ = l_ang_vel;
    left_lin_acc_ = l_lin_acc;
    left_ang_acc_ = l_ang_acc;
    left_jacobian_ = l_jacobian;
    left_bias_acc_ = l_bias_acc;
}


void DifferentialInverseKinematicsQP::set_desired_ee_transform(const Eigen::Transform<double, 3, Eigen::Affine> &r_transform, const Eigen::Transform<double, 3, Eigen::Affine> &l_transform)
{
    right_desired_transform_ = r_transform;
    left_desired_transform_ = l_transform;
}


void DifferentialInverseKinematicsQP::set_desired_ee_twist(const Eigen::Ref<const Eigen::Vector3d> &r_lin_vel, const Eigen::Ref<const Eigen::Vector3d> &r_ang_vel, const Eigen::Ref<const Eigen::Vector3d> &l_lin_vel, const Eigen::Ref<const Eigen::Vector3d> &l_ang_vel)
{
    right_desired_lin_vel_ = r_lin_vel;
    right_desired_ang_vel_ = r_ang_vel;

    left_desired_lin_vel_ = l_lin_vel;
    left_desired_ang_vel_ = l_ang_vel;
}


void DifferentialInverseKinematicsQP::set_desired_ee_acceleration(const Eigen::Ref<const Eigen::Vector3d> &r_lin_acc, const Eigen::Ref<const Eigen::Vector3d> &r_ang_acc, const Eigen::Ref<const Eigen::Vector3d> &l_lin_acc, const Eigen::Ref<const Eigen::Vector3d> &l_ang_acc)
{
    right_desired_lin_acc_ = r_lin_acc;
    right_desired_ang_acc_ = r_ang_acc;

    left_desired_lin_acc_ = l_lin_acc;
    left_desired_ang_acc_ = l_ang_acc;
}

std::optional<Eigen::VectorXd> DifferentialInverseKinematicsQP::solve()
{
    /* --- J_pos_ / J_ori_ con copia sicura (gestisce catene disattive) --- */
    auto safeCopy = [](Eigen::MatrixXd& dst, int r0, int c0,
                       const Eigen::MatrixXd& src, int sr0, int sc0,
                       int r, int c) {
        int rr = std::max(0, std::min(r, (int)dst.rows() - r0));
        int cc = std::max(0, std::min(c, (int)dst.cols() - c0));
        rr = std::max(0, std::min(rr, (int)src.rows() - sr0));
        cc = std::max(0, std::min(cc, (int)src.cols() - sc0));
        if (rr > 0 && cc > 0) {
            dst.block(r0, c0, rr, cc) = src.block(sr0, sc0, rr, cc);
        }
    };

    int nr = right_arm_joints_;
    int nl = left_arm_joints_;
    int nt = torso_joints_;
    int N  = nr + nl + nt;

    // POS
    safeCopy(J_pos_, 0,         0,        right_jacobian_, 0, 0, 3, nr);           // right pos
    safeCopy(J_pos_, 0,         nr+nl,    right_jacobian_, 0, nr, 3, nt);          // torso on right EE
    safeCopy(J_pos_, 3,         nr,       left_jacobian_,  0, 0, 3, nl+nt);        // left pos (+torso)

    // ORI
    safeCopy(J_ori_, 0,         0,        right_jacobian_, 3, 0, 3, nr);           // right ori
    safeCopy(J_ori_, 0,         nr+nl,    right_jacobian_, 3, nr, 3, nt);          // torso on right EE
    safeCopy(J_ori_, 3,         nr,       left_jacobian_,  3, 0, 3, nl+nt);        // left ori (+torso)

    /* --- Errori cartesiani --- */
    Eigen::VectorXd e_p(6);
    e_p <<  right_desired_transform_.translation() - right_transform_.translation(),
            left_desired_transform_.translation()  - left_transform_.translation();

    Eigen::VectorXd e_v(6);
    e_v <<  right_desired_lin_vel_ - right_lin_vel_,
            left_desired_lin_vel_  - left_lin_vel_;

    Eigen::AngleAxisd right_e_o_aa(right_desired_transform_.rotation() * right_transform_.rotation().transpose());
    Eigen::AngleAxisd left_e_o_aa (left_desired_transform_.rotation()  * left_transform_.rotation().transpose());
    Eigen::Vector3d right_e_o = right_e_o_aa.axis() * right_e_o_aa.angle();
    Eigen::Vector3d left_e_o  = left_e_o_aa.axis()  * left_e_o_aa.angle();

    Eigen::VectorXd e_o(6);
    e_o << right_e_o, left_e_o;

    Eigen::VectorXd e_w(6);
    e_w <<  right_desired_ang_vel_ - right_ang_vel_,
            left_desired_ang_vel_  - left_ang_vel_;

    /* --- QP --- */
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(N, N);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(N);

    // Tracking cartesiano
    {
        Eigen::Matrix<double, 6, 1> lin_bias_acc; lin_bias_acc << right_bias_acc_.topRows<3>(), left_bias_acc_.topRows<3>();
        Eigen::Matrix<double, 6, 1> lin_des_acc;  lin_des_acc  << right_desired_lin_acc_,       left_desired_lin_acc_;
        Eigen::Matrix<double, 6, 1> lin_ref = -lin_bias_acc + lin_des_acc + lin_Kp_ * e_p + lin_Kd_ * e_v;

        Eigen::Matrix<double, 6, 1> ang_bias_acc; ang_bias_acc << right_bias_acc_.bottomRows<3>(), left_bias_acc_.bottomRows<3>();
        Eigen::Matrix<double, 6, 1> ang_des_acc;  ang_des_acc  << right_desired_ang_acc_,         left_desired_ang_acc_;
        Eigen::Matrix<double, 6, 1> ang_ref = -ang_bias_acc + ang_des_acc + ang_Kp_ * e_o + ang_Kd_ * e_w;

        P += J_pos_.transpose() * W_lin_cart_ * J_pos_;
        q += -2.0 * J_pos_.transpose() * W_lin_cart_ * lin_ref;

        P += J_ori_.transpose() * W_ang_cart_ * J_ori_;
        q += -2.0 * J_ori_.transpose() * W_ang_cart_ * ang_ref;
    }

    // Joint posture (PD)
    {
        const Eigen::VectorXd des_joints_vel = Eigen::VectorXd::Zero(N);
        const Eigen::VectorXd des_joints_acc = Eigen::VectorXd::Zero(N);
        const Eigen::VectorXd ddq_ref = des_joints_acc
            + joint_pos_p_gain_.asDiagonal() * (joint_ref_ - joints_)
            + joint_pos_d_gain_.asDiagonal() * (des_joints_vel - joints_vel_);

        P += joint_pos_weights_.asDiagonal();
        q += -2.0 * joint_pos_weights_.asDiagonal() * ddq_ref;
    }

    /* --- Add limiter contribution to the constraint --- */
    Eigen::MatrixXd G_limiter;
    Eigen::VectorXd h_limiter;
    std::tie(G_limiter, h_limiter) = limit_constraint(joints_, joints_lower_limits_, joints_upper_limits_, joints_limits_gains_);

    /* --- QP Resolution --- */
    qp_results_ = solve_qp(P, q, G_limiter, h_limiter);
    return qp_results_;
}

std::optional<Eigen::VectorXd> DifferentialInverseKinematicsQP::solve_qp(
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

    Eigen::MatrixXd G_l(joints.size() * 2 , joints.size());
    Eigen::VectorXd h_l(joints.size() * 2 );

    /*
    * Implement limits as described in:
    * Kanoun, Oussama. "Real-time prioritized kinematic control under inequality constraints for redundant manipulators." Robotics: Science and Systems. Vol. 7. 2012.
    *
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

    G_l.block(0, 0, joints.size(), joints.size()) = Eigen::MatrixXd::Identity(joints.size(), joints.size());
    h_l.block(0, 0, joints.size(), 1) =             gains.asDiagonal() * ((joints_upper_limit - joints)/sampling_time_ - joints_vel_)/sampling_time_;

    G_l.block(joints.size(), 0, joints.size(), joints.size()) = -Eigen::MatrixXd::Identity(joints.size(), joints.size());
    h_l.block(joints.size(), 0, joints.size(), 1) =             -1.0 * gains.asDiagonal() * ((joints_lower_limit - joints)/sampling_time_ - joints_vel_)/sampling_time_;

    /* Maximize manipulability term
    *
    * m = sqrt(det(J J^T))
    *
    * We impose a constraint on the time derivative of the manipulability measure:
    *
    * dm/dt >= - a_m * (m - m*), a_m > 0
    *
    * which is equivalent to a LTI system converging to m*. We are solving for the acceleration, so:
    *
    * ddq^ = optimal acceleration.
    * dq^ = ddq^ * dt + dq;
    * dm^/dt = dm/dq * dq^ = dm/dq * (ddq^ * dt + dq);
    *
    * dm^/dt >= - a_m * (m - m*) -> dm/dq * ddq^ * dt >= - a_m * (m^ - m*) - dm/dq * dq
    *
    * Let's rearrange the equation:
    *
    * (-dm/dq * dt) * ddq^ <= a_m * (m^ - m*) + dm/dq * dq
    *
    * Notice that for a_m = 0 we are just imposing that the manipulability measure cannot decrease, which is not the best choice
    * as we can accept a reduction of the manipulability measure as long as it is necessary to reach some pose.
    *
    */

    auto addManipConstraint = [&](const Eigen::MatrixXd& jacobian) {
        G_l.conservativeResize(G_l.rows() + 1, G_l.cols());
        h_l.conservativeResize(h_l.size() + 1);

        Eigen::Matrix<double,6,6> JJT = jacobian * jacobian.transpose();
        const double manip = std::sqrt(std::max(0.0, JJT.determinant()));

        Eigen::LDLT<Eigen::Matrix<double,6,6>> JJT_ldlt(JJT);
        Eigen::VectorXd dmdq(joints_.size());
        for(int i = 0; i < joints_.size(); i++){
            if(i == 0){dmdq(i) = 0.0;}
            else{dmdq(i) = manip/2 * (JJT_ldlt.solve( partial_derivative(jacobian,i)*jacobian.transpose() )).trace();}
        }
        
        G_l.bottomRows(1) = -sampling_time_ * dmdq.transpose();
        h_l(h_l.size() - 1) = dmdq.dot(joints_vel_);

        if(manip >= improve_manip_th_)  h_l(h_l.size() - 1) += improve_manip_dyn_* (manip - improve_manip_th_);
    };

    if(right_arm_joints_ > 0){

        Eigen::MatrixXd full_right_jac = Eigen::MatrixXd::Zero(6, joints_.size());
        full_right_jac.topRows(3)    = J_pos_.topRows(3);
        full_right_jac.bottomRows(3) = J_ori_.topRows(3);
        addManipConstraint(full_right_jac);
    }

    if(left_arm_joints_ > 0){
        Eigen::MatrixXd full_left_jac  = Eigen::MatrixXd::Zero(6, joints_.size());
        full_left_jac.topRows(3)     = J_pos_.bottomRows(3);
        full_left_jac.bottomRows(3)  = J_ori_.bottomRows(3);

        addManipConstraint(full_left_jac);
    }

    return std::make_tuple(G_l, h_l);
}

Eigen::MatrixXd DifferentialInverseKinematicsQP::partial_derivative(const Eigen::MatrixXd &J, const unsigned int jointNum)
{
    /*
    * J. Haviland, P. Corke - "A Systematic Approach to Computing the Manipulator Jacobian and Hessian using the Elementary Transform Sequence"
    * IX. THE MANIPULATOR HESSIAN - Eqs. (64) and (67)
    */

	Eigen::MatrixXd dJ(6,joints_.size());

	for(int i = 0; i < joints_.size(); i++)
	{
		if (jointNum < i)
		{
			// a_j x (a_i x a_i)
			dJ(0,i) = J(4,jointNum)*J(2,i) - J(5,jointNum)*J(1,i);
			dJ(1,i) = J(5,jointNum)*J(0,i) - J(3,jointNum)*J(2,i);
			dJ(2,i) = J(3,jointNum)*J(1,i) - J(4,jointNum)*J(0,i);

			// a_j x a_i
			dJ(3,i) = J(4,jointNum)*J(5,i) - J(5,jointNum)*J(4,i);
			dJ(4,i) = J(5,jointNum)*J(3,i) - J(3,jointNum)*J(5,i);
			dJ(5,i) = J(3,jointNum)*J(4,i) - J(4,jointNum)*J(3,i);
		}
		else
		{
			// a_i x (a_j x a_j)
			dJ(0,i) = J(4,i)*J(2,jointNum) - J(5,i)*J(1,jointNum);
			dJ(1,i) = J(5,i)*J(0,jointNum) - J(3,i)*J(2,jointNum);
			dJ(2,i) = J(3,i)*J(1,jointNum) - J(4,i)*J(0,jointNum);

			dJ(3,i) = 0.0;
			dJ(4,i) = 0.0;
			dJ(5,i) = 0.0;
		}
	}

	return dJ;
}

#endif /* DIFFERENTIAL_INVERSE_KINEMATICS_QP_CPP */
