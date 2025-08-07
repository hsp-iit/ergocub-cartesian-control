/*
 * Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * MIT license. See the accompanying LICENSE file for details.
 */

#ifndef DIFFERENTIAL_INVERSE_KINEMATICS_QP_H
#define DIFFERENTIAL_INVERSE_KINEMATICS_QP_H

#include <proxsuite/proxqp/dense/dense.hpp>

/**
 * Template class representing the differential inverse kinematics of a kinematic chain solved via Quadratic Programming (QP).
 * The joint limitation mechanism is represented using the template parameter LimiterType.
 */
class DifferentialInverseKinematicsQP
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    /**
     * Constructor.
     * @param sampling_time The sampling time used for the control loop.
     * @param verbose To print some info.
     * @param limits_param The limit constraints gain.
     * @param joint_acc_weight The weight used in the minimization of the QP functional for the joint velocities term.
     * @param position_param The weight and gain used in the minimization of the QP functional for the position term.
     * @param orientation_param The weight and gain used in the minimization of the QP functional for the orientation term.
     * @param joint_pos_param The weight and gain used in the minimization of the QP functional for the joint positions term.
     * @param torso_pos_param The weight and gain used in the minimization of the QP functional for the torso positions term.
     * @param joint_ref The joint position reference vector.
     */
    DifferentialInverseKinematicsQP(
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
        const Eigen::VectorXd &joint_ref);

    ~DifferentialInverseKinematicsQP() = default;

    /**
     * Update the internal quantities of the inverse kinematics.
     * Specifically, this concrete implementation evaluates the matrices \f$ P \f$, \f$ A \f$ and the vectors \f$ q \f$ and \f$ b \f$
     * of the QP inverse kinematics problem and call the method DifferentialInverseKinematicsQP::solve_qp() with those matrices.
     * The method DifferentialInverseKinematicsQP::solve_qp() can be overriden by the inheritee in case.
     * @return true if the update was succesful, false otherwise.
     * @warning This method should be called only after setting the current robot state via DifferentialInverseKinematics::set_robot_state()
     *          and the desired end-effector transform via DifferentialInverseKinematics::set_desired_ee_transform().
     */
    std::optional<Eigen::VectorXd> solve();

    void set_robot_state(   const Eigen::Ref<const Eigen::VectorXd> &joints,
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
                            const Eigen::Ref<const Eigen::VectorXd> &l_bias_acc);

    void set_desired_ee_transform(const Eigen::Transform<double, 3, Eigen::Affine> &r_transform, const Eigen::Transform<double, 3, Eigen::Affine> &l_transform);

    void set_desired_ee_twist(const Eigen::Ref<const Eigen::Vector3d> &r_lin_vel = Eigen::Vector3d::Zero(), const Eigen::Ref<const Eigen::Vector3d> &r_ang_vel = Eigen::Vector3d::Zero(), const Eigen::Ref<const Eigen::Vector3d> &l_lin_vel = Eigen::Vector3d::Zero(), const Eigen::Ref<const Eigen::Vector3d> &l_ang_vel = Eigen::Vector3d::Zero());

    void set_desired_ee_acceleration(const Eigen::Ref<const Eigen::Vector3d> &r_lin_acc = Eigen::Vector3d::Zero(), const Eigen::Ref<const Eigen::Vector3d> &r_ang_acc = Eigen::Vector3d::Zero(), const Eigen::Ref<const Eigen::Vector3d> &l_lin_acc = Eigen::Vector3d::Zero(), const Eigen::Ref<const Eigen::Vector3d> &l_ang_acc = Eigen::Vector3d::Zero());

    bool set_joint_limits(
        const Eigen::Ref<const Eigen::VectorXd> &lower_limits,
        const Eigen::Ref<const Eigen::VectorXd> &upper_limits,
        const Eigen::Ref<const Eigen::VectorXd> &gains);

protected:
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
    std::tuple<Eigen::MatrixXd, Eigen::VectorXd> limit_constraint(
        const Eigen::Ref<const Eigen::VectorXd> &joints,
        const Eigen::Ref<const Eigen::VectorXd> &joints_lower_limit,
        const Eigen::Ref<const Eigen::VectorXd> &joints_upper_limit,
        const Eigen::Ref<const Eigen::VectorXd> &gains);

    /**
     * Solve the the Quadratic Programming (QP) problem:
     * \f[
     *  \begin{split}
     *  \begin{array}{ll}
     *  \underset{x}{\mbox{minimize}}
     *      & \frac{1}{2} x^T P x + q^T x \\
     *  \mbox{subject to}
     *      & G x \preccurlyeq h\\
     *  \end{array}
     *  \end{split}
     * \f]
     *@param P The matrix \f$P\f$ of the above problem.
     *@param q The vector \f$q\f$ of the above problem.
     *@param G The matrix \f$G\f$ of the above problem.
     *@param h The vector \f$h\f$ of the above problem.
     *@return A tuple containing the status of the solution and the solution itself in vectorial form.
     */
    std::optional<Eigen::VectorXd> solve_qp(
        const Eigen::Ref<const Eigen::MatrixXd> &P,
        const Eigen::Ref<const Eigen::VectorXd> &q,
        const Eigen::Ref<const Eigen::MatrixXd> &G,
        const Eigen::Ref<const Eigen::VectorXd> &h);

private:

    const double sampling_time_;
    const bool verbose_;
    const int right_arm_joints_;
    const int left_arm_joints_;
    const int torso_joints_;
    const Eigen::VectorXd joint_acc_weight_;
    const Eigen::VectorXd joint_pos_weights_, joint_pos_p_gain_, joint_pos_d_gain_;
    const Eigen::VectorXd cartesian_pos_weight_, cartesian_pos_p_gain_, cartesian_pos_d_gain_;
    const Eigen::VectorXd cartesian_ori_weight_, cartesian_ori_p_gain_, cartesian_ori_d_gain_;
    const Eigen::VectorXd joint_ref_;
    const double improve_manip_weight_;

    Eigen::Transform<double, 3, Eigen::Affine> right_transform_, left_transform_, right_desired_transform_, left_desired_transform_;
    Eigen::MatrixXd right_jacobian_, left_jacobian_;
    Eigen::VectorXd right_bias_acc_, left_bias_acc_;

    Eigen::DiagonalMatrix<double, 6> W_lin_cart_, lin_Kp_, lin_Kd_, W_ang_cart_, ang_Kp_, ang_Kd_;

    Eigen::VectorXd joints_, joints_vel_, joints_lower_limits_, joints_upper_limits_, joints_limits_gains_;
    Eigen::Vector3d right_lin_vel_, right_ang_vel_, right_lin_acc_, right_ang_acc_;
    Eigen::Vector3d right_desired_lin_vel_, right_desired_ang_vel_, right_desired_lin_acc_, right_desired_ang_acc_;
    Eigen::Vector3d left_lin_vel_, left_ang_vel_, left_lin_acc_, left_ang_acc_;
    Eigen::Vector3d left_desired_lin_vel_, left_desired_ang_vel_, left_desired_lin_acc_, left_desired_ang_acc_;

    std::optional<Eigen::VectorXd> qp_results_;

    double right_max_manip_, right_manip_, right_weight_manip_function_;
    double left_max_manip_, left_manip_, left_weight_manip_function_;

    proxsuite::proxqp::Results<double> old_solution_;

    Eigen::MatrixXd partial_derivative(const Eigen::MatrixXd &J, const unsigned int jointNum);

    const std::string class_name_ = "DifferentialInverseKinematicsQP";
};

/* Matrix manipulation utils*/
void addBlockOnDiag(Eigen::MatrixXd &M, const Eigen::MatrixXd &M_add);

#endif /* DIFFERENTIAL_INVERSE_KINEMATICS_QP_H */
