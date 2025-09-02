// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef DIFFERENTIAL_INVERSE_KINEMATICS_QP_H
#define DIFFERENTIAL_INVERSE_KINEMATICS_QP_H

#include <DifferentialInverseKinematics.h>
#include <proxsuite/proxqp/dense/dense.hpp>

/**
 * Template class representing the differential inverse kinematics of a kinematic chain solved via Quadratic Programming (QP).
 * The joint limitation mechanism is represented using the template parameter LimiterType.
 */
class DifferentialInverseKinematicsQP : public DifferentialInverseKinematics
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    /**
     * Constructor.
     * @param sampling_time The sampling time used for the control loop.
     * @param limits_param The limit constraints gain.
     * @param joint_acc_weight The weight used in the minimization of the QP functional for the joint velocities term.
     * @param position_param The weight and gain used in the minimization of the QP functional for the position term.
     * @param orientation_param The weight and gain used in the minimization of the QP functional for the orientation term.
     * @param joint_pos_param The weight and gain used in the minimization of the QP functional for the joint positions term.
     * @param joint_ref The joint position reference vector.
    * @param torso_joints_to_stiffen The number of the torso joints to increase the PID of to make torso motion.
     * @param verbose To print some info.
     * @param improve_manip_dyn The dynamic gain for the manipulability improvement.
     * @param improve_manip_th The threshold for the manipulability improvement
     */
    DifferentialInverseKinematicsQP(
        double sampling_time,
        double limits_param,
        const Eigen::VectorXd & joint_acc_weight,
        const Eigen::VectorXd &position_param,
        const Eigen::MatrixXd &orientation_param,
        const Eigen::VectorXd &joint_pos_param,
        const Eigen::MatrixXd &joint_ref,
        double improve_manip_dyn,
        double improve_manip_th,
        const int torso_joints_to_stiffen,
        bool verbose);

    virtual ~DifferentialInverseKinematicsQP() = default;

    /**
     * Update the internal quantities of the inverse kinematics.
     * Specifically, this concrete implementation evaluates the matrices \f$ P \f$, \f$ A \f$ and the vectors \f$ q \f$ and \f$ b \f$
     * of the QP inverse kinematics problem and call the method DifferentialInverseKinematicsQP::solve() with those matrices.
     * The method DifferentialInverseKinematicsQP::solve() can be overriden by the inheritee in case.
     * @return true if the update was succesful, false otherwise.
     * @warning This method should be called only after setting the current robot state via DifferentialInverseKinematics::set_robot_state()
     *          and the desired end-effector transform via DifferentialInverseKinematics::set_desired_ee_transform().
     */
    virtual std::optional<Eigen::VectorXd> eval_reference_velocities() override;

    virtual void set_robot_state(
        const Eigen::Ref<const Eigen::VectorXd> &joints,
        const Eigen::Ref<const Eigen::VectorXd> &joints_vel,
        const Eigen::Transform<double, 3, Eigen::Affine> &transform,
        const Eigen::Ref<const Eigen::MatrixXd> &jacobian,
        const Eigen::Ref<const Eigen::VectorXd> &bias_acc) override;

    virtual void set_desired_ee_transform(const Eigen::Transform<double, 3, Eigen::Affine> &transform) override;

    virtual void set_desired_ee_twist(const Eigen::Ref<const Eigen::Vector3d> &lin_vel, const Eigen::Ref<const Eigen::Vector3d> &ang_vel) override;

    virtual void set_joint_limits(
        const Eigen::Ref<const Eigen::VectorXd> &lower_limits,
        const Eigen::Ref<const Eigen::VectorXd> &upper_limits,
        const Eigen::Ref<const Eigen::VectorXd> &gains) override;

    // /* ---- Manipulability gains ---- */
    // void setManipImproveGains(double dyn, double th);
    // double getManipImproveDyn() const { return improve_manip_dyn_; }
    // double getManipImproveTh()  const { return improve_manip_th_;  }

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
    std::optional<Eigen::VectorXd> solve(
        const Eigen::Ref<const Eigen::MatrixXd> &P,
        const Eigen::Ref<const Eigen::VectorXd> &q,
        const Eigen::Ref<const Eigen::MatrixXd> &G,
        const Eigen::Ref<const Eigen::VectorXd> &h);

    /**
     * Solve the the Quadratic Programming (QP) problem:
     * \f[
     *  \begin{split}
     *  \begin{array}{ll}
     *  \underset{x}{\mbox{minimize}}
     *      & \frac{1}{2} x^T P x + q^T x \\
     *  \mbox{subject to}
     *      & A x = b \\
     *      & G x \preccurlyeq h\\
     *  \end{array}
     *  \end{split}
     * \f]
     *@param P The matrix \f$P\f$ of the above problem.
     *@param q The vector \f$q\f$ of the above problem.
     *@param A The matrix \f$A\f$ of the above problem.
     *@param b The vector \f$b\f$ of the above problem.
     *@param G The matrix \f$G\f$ of the above problem.
     *@param h The vector \f$h\f$ of the above problem.
     *@return A tuple containing the status of the solution and the solution itself in vectorial form.
     */
    std::optional<Eigen::VectorXd> solve(
        const Eigen::Ref<const Eigen::MatrixXd> &P,
        const Eigen::Ref<const Eigen::VectorXd> &q,
        const Eigen::Ref<const Eigen::MatrixXd> &A,
        const Eigen::Ref<const Eigen::VectorXd> &b,
        const Eigen::Ref<const Eigen::MatrixXd> &G,
        const Eigen::Ref<const Eigen::VectorXd> &h);

    Eigen::MatrixXd partial_derivative(const Eigen::MatrixXd &J, const unsigned int jointNum);

private:
    Eigen::VectorXd joints_, joints_vel_, joints_lower_limits_, joints_upper_limits_, joints_limits_gains_;

    Eigen::Transform<double, 3, Eigen::Affine> transform_;

    Eigen::VectorXd bias_acc_;

    Eigen::Transform<double, 3, Eigen::Affine> desired_transform_;

    Eigen::Vector3d desired_lin_vel_, desired_ang_vel_;

    Eigen::MatrixXd jacobian_;

    std::optional<Eigen::VectorXd> reference_joints_velocities_;

    const int slack_size_ = 3;

    const double sampling_time_;

    const double limits_param_;

    const Eigen::VectorXd joint_acc_weight_;

    const Eigen::VectorXd position_param_;

    const Eigen::MatrixXd orientation_param_;

    const Eigen::VectorXd torso_param_;

    const Eigen::MatrixXd torso_ref_;

    const Eigen::VectorXd joint_pos_param_;

    const Eigen::MatrixXd joint_ref_;

    int torso_joints_to_stiffen_;

    double max_manip_, manip_, weight_manip_function_;
    
    double improve_manip_dyn_;
    double improve_manip_th_;
    proxsuite::proxqp::Results<double> old_solution_;

    bool verbose_;

    

    const std::string class_name_ = "DifferentialInverseKinematicsQP";
};

/* Matrix manipulation utils*/
void addBlockOnDiag(Eigen::MatrixXd &M, const Eigen::MatrixXd &M_add);

#endif /* DIFFERENTIAL_INVERSE_KINEMATICS_QP_H */
