/*
 * Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * MIT license. See the accompanying LICENSE file for details.
 */

#ifndef DIFFERENTIAL_INVERSE_KINEMATICS_QP_CPP
#define DIFFERENTIAL_INVERSE_KINEMATICS_QP_CPP

#include <DifferentialInverseKinematicsQP.h>

#include <yarp/os/LogStream.h>

#include <Eigen/Dense>

using namespace proxsuite::proxqp;

DifferentialInverseKinematicsQP::DifferentialInverseKinematicsQP(
    const double sampling_time,
    const bool verbose,
    const int right_arm_joints,
    const int left_arm_joints,
    const int torso_joints,
    const Eigen::VectorXd &joint_acc_weight,
    const Eigen::VectorXd &joint_pos_weights,
    const Eigen::VectorXd &joint_pos_p_gain,
    const Eigen::VectorXd &joint_pos_d_gain,
    const Eigen::VectorXd &cartesian_pos_weight,
    const Eigen::VectorXd &cartesian_pos_p_gain,
    const Eigen::VectorXd &cartesian_pos_d_gain,
    const Eigen::VectorXd &cartesian_ori_weight,
    const Eigen::VectorXd &cartesian_ori_p_gain,
    const Eigen::VectorXd &cartesian_ori_d_gain,
    const double improve_manip_weight,
    const Eigen::VectorXd &joint_ref):
    sampling_time_(sampling_time),
    verbose_(verbose),
    right_arm_joints_(right_arm_joints),
    left_arm_joints_(left_arm_joints),
    torso_joints_(torso_joints),
    joint_acc_weight_(joint_acc_weight),
    joint_pos_weights_(joint_pos_weights),
    joint_pos_p_gain_(joint_pos_p_gain),
    joint_pos_d_gain_(joint_pos_d_gain),
    cartesian_pos_weight_(cartesian_pos_weight),
    cartesian_pos_p_gain_(cartesian_pos_p_gain),
    cartesian_pos_d_gain_(cartesian_pos_d_gain),
    cartesian_ori_weight_(cartesian_ori_weight),
    cartesian_ori_p_gain_(cartesian_ori_p_gain),
    cartesian_ori_d_gain_(cartesian_ori_d_gain),
    improve_manip_weight_(improve_manip_weight),
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
    left_desired_ang_acc_(Eigen::Vector3d::Zero()),
    right_max_manip_(0.0),
    right_manip_(0.0),
    right_weight_manip_function_(0.0),
    left_max_manip_(0.0),
    left_manip_(0.0),
    left_weight_manip_function_(0.0)
{
    std::string error_message = "[" + class_name_ + "::ctor]. Error: parameter(s) not valid:";
    bool error = false;

    if (sampling_time_ <= 0)
    {
        error = true;
        error_message += " sampling_time";
    }

    int active_joints_num = right_arm_joints + left_arm_joints + torso_joints;

    auto nonPositiveCheck = [](const Eigen::VectorXd& vec)
    {
        for(int i=0; i<vec.size(); i++)
        {   
            if(vec(i)<=0)
            {
                return true;
            }
        }

        return false;
    };

    if(nonPositiveCheck(joint_acc_weight))
    {
        error = true;
        error_message += " joint_acc_weight";
    }

    if(nonPositiveCheck(joint_pos_weights) || (joint_pos_weights.size() != active_joints_num))
    {
        error = true;
        error_message += " joint_pos_weights";
    }

    if(nonPositiveCheck(joint_pos_p_gain) || (joint_pos_p_gain.size() != active_joints_num))
    {   
        error = true;
        error_message += " joint_pos_p_gain";
    }

    if(nonPositiveCheck(joint_pos_d_gain) || (joint_pos_d_gain.size() != active_joints_num))
    {   
        error = true;
        error_message += " joint_pos_d_gain";
    }

    if(nonPositiveCheck(cartesian_pos_weight))
    {
        error = true;
        error_message += " cartesian_pos_weight";
    }

    if(nonPositiveCheck(cartesian_pos_p_gain))
    {   
        error = true;
        error_message += " cartesian_pos_p_gain";
    }

    if(nonPositiveCheck(cartesian_pos_d_gain))
    {   
        error = true;
        error_message += " cartesian_pos_d_gain";
    }

    if(nonPositiveCheck(cartesian_ori_weight))
    {   
        error = true;
        error_message += " cartesian_ori_weight";
    }

    if(nonPositiveCheck(cartesian_ori_p_gain))
    {   
        error = true;
        error_message += " cartesian_ori_p_gain";
    }

    if(nonPositiveCheck(cartesian_ori_d_gain))
    {   
        error = true;
        error_message += " cartesian_ori_d_gain";
    }

    if(improve_manip_weight<0)
    {
        error = true;
        error_message += " improve_manip_weight";
    }

    if(joint_ref.size() != active_joints_num)
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
    // joints are in the from [right_arm left_arm torso]

    joints_lower_limits_ = lower_limits;
    joints_upper_limits_ = upper_limits;

    for(int i=0; i<gains.size(); i++)
    {   
        if(gains(i)<0 || gains(i)>1.0)
        {
            std::cerr<<"[" + class_name_ + "::set_joint_limits] At least one value of gains outside the range [0,1]";
            return false;
        }
    }

    joints_limits_gains_.resize(right_arm_joints_ + left_arm_joints_ + torso_joints_);

    for(int i=0; i<right_arm_joints_; i++)
    {
        joints_limits_gains_(i) = gains(0);
    }
    for(int i=right_arm_joints_; i<right_arm_joints_ + left_arm_joints_; i++)
    {
        joints_limits_gains_(i) = gains(1);
    }
    for(int i=right_arm_joints_ + left_arm_joints_; i<right_arm_joints_ + left_arm_joints_ + torso_joints_; i++)
    {
        joints_limits_gains_(i) = gains(2);
    }


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
    // joints are in the from [right_arm left_arm torso]
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
    /*Manips update*/
    right_manip_ = sqrt((right_jacobian_ * right_jacobian_.transpose()).determinant());
    right_max_manip_ = std::max(right_manip_,right_max_manip_);
    right_weight_manip_function_ = pow(1 - (right_manip_ / right_max_manip_), 2);

    left_manip_ = sqrt((left_jacobian_ * left_jacobian_.transpose()).determinant());
    left_max_manip_ = std::max(left_manip_,left_max_manip_);
    left_weight_manip_function_ = pow(1 - (left_manip_ / left_max_manip_), 2);
        
    /* JACOBIANS
        Jx =|Jxpos|   , x= r,l
            |Jxori|
        J_y =   |Jry(3xnr)          O   Jry(3xnt)|  y = pos, ori
                |O          Jly(3xnl)   Jly(3xnt)|
     */

    Eigen::MatrixXd J_pos = Eigen::MatrixXd::Zero(6, right_arm_joints_ + left_arm_joints_ + torso_joints_);
    Eigen::MatrixXd J_ori = Eigen::MatrixXd::Zero(6, right_arm_joints_ + left_arm_joints_ + torso_joints_);

    J_pos.topLeftCorner(3,right_arm_joints_) = right_jacobian_.topLeftCorner(3,right_arm_joints_);
    J_pos.topRightCorner(3,torso_joints_) = right_jacobian_.topRightCorner(3,torso_joints_);
    J_pos.bottomRightCorner(3,left_arm_joints_ + torso_joints_) = left_jacobian_.topLeftCorner(3,left_arm_joints_ + torso_joints_);

    J_ori.topLeftCorner(3,right_arm_joints_) = right_jacobian_.bottomLeftCorner(3,right_arm_joints_);
    J_ori.topRightCorner(3,torso_joints_) = right_jacobian_.bottomRightCorner(3,torso_joints_);
    J_ori.bottomRightCorner(3,left_arm_joints_ + torso_joints_) = left_jacobian_.bottomLeftCorner(3,left_arm_joints_ + torso_joints_);

    /* Position error. */
    Eigen::VectorXd e_p(6);
    e_p <<  right_desired_transform_.translation() - right_transform_.translation(),
            left_desired_transform_.translation() - left_transform_.translation();

    /* Linear velocity error. */
    Eigen::VectorXd e_v(6);
    e_v <<  right_desired_lin_vel_ - right_lin_vel_,
            left_desired_lin_vel_ - left_lin_vel_;

    /* Orientation error. */
    Eigen::AngleAxisd right_e_o_aa(right_desired_transform_.rotation() * right_transform_.rotation().transpose());
    Eigen::Vector3d right_e_o = right_e_o_aa.axis() * right_e_o_aa.angle();
    Eigen::AngleAxisd left_e_o_aa(left_desired_transform_.rotation() * left_transform_.rotation().transpose());
    Eigen::Vector3d left_e_o = left_e_o_aa.axis() * left_e_o_aa.angle();
    Eigen::VectorXd e_o(6);
    e_o << right_e_o, left_e_o;

    /* Angular velocity error. */
    Eigen::VectorXd e_w(6);
    e_w <<  right_desired_ang_vel_ - right_ang_vel_,
            left_desired_ang_vel_ - left_ang_vel_;

    /* Setup the QP-related matrices. */

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(joints_.size(), joints_.size());
    Eigen::MatrixXd q = Eigen::VectorXd::Zero(joints_.size());


    {   /*  Follow cartesian reference
        *  min || REF - J ddq_||^2_W =  min ddq_^T J^T W J ddq_ - 2 ddq_^T J^T W REF
        *  ddq_                         ddq_
        */

        /* Cartesian reference values.
        *   REF = - dJ*dq + ddx_des + Kp * (x_des - x) + Kd * (dx_des - dx) ,   x = pos, ori
        */

        // Linear reference
        Eigen::Vector<double, 6> lin_bias_acc; lin_bias_acc << right_bias_acc_.topRows<3>(), left_bias_acc_.topRows<3>();
        Eigen::Vector<double, 6> lin_des_acc; lin_des_acc << right_desired_lin_acc_, left_desired_lin_acc_;

        Eigen::Vector<double, 6> lin_ref = -lin_bias_acc + lin_des_acc + lin_Kp_ * e_p + lin_Kd_ * e_v;

        // Angular reference
        Eigen::Vector<double, 6> ang_bias_acc; ang_bias_acc << right_bias_acc_.bottomRows<3>(), left_bias_acc_.bottomRows<3>();
        Eigen::Vector<double, 6> ang_des_acc; ang_des_acc << right_desired_ang_acc_, left_desired_ang_acc_;

        Eigen::Vector<double, 6> ang_ref = -ang_bias_acc + ang_des_acc + ang_Kp_ * e_o + ang_Kd_ * e_w;

        P += J_pos.transpose() * W_lin_cart_ * J_pos;
        q += - 2.0 * J_pos.transpose() * W_lin_cart_ * lin_ref;

        P += J_ori.transpose() * W_ang_cart_ * J_ori;
        q += - 2.0 * J_ori.transpose() * W_ang_cart_ * ang_ref;
    }

    {   /* Keep joints in joint_ref position
        *  min || ddq_ref - ddq_||^2_W =  min ddq_^T W ddq_ - 2 ddq_^T W ddq_ref
        *  ddq_                           ddq_
        */

        const Eigen::VectorXd des_joints_vel = Eigen::VectorXd::Zero(joints_.size());
        const Eigen::VectorXd des_joints_acc = Eigen::VectorXd::Zero(joints_.size());
        const Eigen::VectorXd ddq_ref = des_joints_acc + joint_pos_p_gain_.asDiagonal() * (joint_ref_ - joints_) + joint_pos_d_gain_.asDiagonal() * (des_joints_vel - joints_vel_);

        P += joint_pos_weights_.asDiagonal();
        q += - 2.0 * joint_pos_weights_.asDiagonal() * ddq_ref;
    }

    {   /*  Restrain joint acceleration (with variable weight)
        *  min ||ddq_||^2_W =  min ddq_^T W ddq_
        *  ddq_                ddq_
        */

        Eigen::MatrixXd W_joint = Eigen::MatrixXd::Identity(joints_.size(),joints_.size());
        W_joint.topLeftCorner(right_arm_joints_,right_arm_joints_)*=joint_acc_weight_(0)*right_weight_manip_function_;
        W_joint.topLeftCorner(right_arm_joints_+left_arm_joints_,right_arm_joints_+left_arm_joints_).bottomRightCorner(left_arm_joints_,left_arm_joints_)*=joint_acc_weight_(1)*left_weight_manip_function_;
        W_joint.bottomRightCorner(torso_joints_,torso_joints_)*=joint_acc_weight_(2)*std::max(right_weight_manip_function_, left_weight_manip_function_);
        P += W_joint;
    }

    {   /*  Improve manipulability (with variable weight)
        *  min || ddq_ref - ddq_||^2_W =  min ddq_^T W ddq_ - 2 ddq_^T W ddq_ref
        *  ddq_                           ddq_
        *
        * How to evaluate ddq_ref? The time derivative of the manipulability index is expressed as:
        *
        * dm/dt = d(m)/dq * dq/dt
        *
        * So, if we were to optimize with respect to the joint velocities dq_:
        *
        * min || dq_ref - dq_||^2_W
        * dp_
        *
        * dq_ref = Km * d(m)/dq, Km > 0
        *
        * i.e. we want the desired joint velocities to be directed as the gradient of the manipulability index, so to make this latter grows.
        *
        * In terms of acceleration:
        *  min || dq_ref - dq_||^2_W = min ||(Km * d(m)/dq) - (ddq_ * dt + dq)||^2_W = min ||(Km * d(m)/dq - dq)/dt - ddq_||^2_W = min ||ddq_ref - ddq_||^2_W
        *  dq_                         ddq_                                            ddq_                                        ddq_
        *
        */

        Eigen::MatrixXd full_right_jacobian = Eigen::MatrixXd::Zero(6, joints_.size());
        Eigen::MatrixXd full_left_jacobian = Eigen::MatrixXd::Zero(6, joints_.size());
        full_right_jacobian.topRows(3) = J_pos.topRows(3);
        full_right_jacobian.bottomRows(3) = J_ori.topRows(3);
        full_left_jacobian.topRows(3) = J_pos.bottomRows(3);
        full_left_jacobian.bottomRows(3) = J_ori.bottomRows(3);
        double full_right_manip = sqrt((full_right_jacobian * full_right_jacobian.transpose()).determinant());
        double full_left_manip = sqrt((full_left_jacobian * full_left_jacobian.transpose()).determinant());
        Eigen::LDLT<Eigen::Matrix<double,6,6>> JJTright_decomp(full_right_jacobian * full_right_jacobian.transpose());
        Eigen::LDLT<Eigen::Matrix<double,6,6>> JJTleft_decomp(full_left_jacobian * full_left_jacobian.transpose());
        Eigen::VectorXd dmdq_right(joints_.size());
        Eigen::VectorXd dmdq_left(joints_.size());

        for(int i = 0; i < joints_.size(); i++)
        {
            dmdq_right(i) = full_right_manip * (JJTright_decomp.solve( partial_derivative(full_right_jacobian,i)*full_right_jacobian.transpose() )).trace();
            dmdq_left(i) = full_left_manip   * (JJTleft_decomp.solve( partial_derivative(full_left_jacobian,i)*full_left_jacobian.transpose() )).trace();
        }
        Eigen::VectorXd ddq_ref_r = (dmdq_right - joints_vel_)/sampling_time_;
        Eigen::VectorXd ddq_ref_l = (dmdq_left - joints_vel_)/sampling_time_;

        Eigen::MatrixXd W_m = Eigen::MatrixXd::Identity(joints_.size(),joints_.size()) * improve_manip_weight_ * std::max(right_weight_manip_function_, left_weight_manip_function_);

        P += W_m;
        q += - 2.0 * W_m * ddq_ref_r;
        P += W_m;
        q += - 2.0 * W_m * ddq_ref_l;
    }

    /* Add limiter contribution to the constraint.*/
    Eigen::MatrixXd G_limiter;
    Eigen::VectorXd h_limiter;
    std::tie(G_limiter, h_limiter) = limit_constraint(joints_, joints_lower_limits_, joints_upper_limits_, joints_limits_gains_);

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

    Eigen::MatrixXd G_l(joints.size() * 2, joints.size());
    Eigen::VectorXd h_l(joints.size() * 2);

    G_l.topRows(joints.size()) = Eigen::MatrixXd::Identity(joints.size(), joints.size());
    G_l.bottomRows(joints.size()) = -Eigen::MatrixXd::Identity(joints.size(), joints.size());

    h_l.head(joints.size()) =        gains.asDiagonal() * ((joints_upper_limit - joints)/sampling_time_ - joints_vel_)/sampling_time_;
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
