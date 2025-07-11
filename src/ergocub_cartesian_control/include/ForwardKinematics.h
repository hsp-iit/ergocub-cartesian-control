// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <Eigen/Dense>

/**
 * Abstract class representing the forward kinematics of a kinematic chain.
 */
class ForwardKinematics
{
public:

    /**
     * Destructor.
     */
    virtual ~ForwardKinematics() = default;

   /**
    * Update the internal quantities of the forward kinematics.
    * @warning This method should be called only after setting the current joints state via ForwardKinematics::set_joints_state().
    */
    virtual void update() = 0;

   /**
    * Set the current joints state.
    * @param joints_pos A Eigen::VectorXd containing the joint positions in radian.
    * @param joints_vel A Eigen::VectorXd containing the joint velocity in radian/sec.
    * @param joints_acc A Eigen::VectorXd containing the joint accelerations in radian/sec^2.
    */
    virtual void set_joints_state(const Eigen::Ref<const Eigen::VectorXd>& joints_pos, const Eigen::Ref<const Eigen::VectorXd>& joints_vel, const Eigen::Ref<const Eigen::VectorXd>& joints_acc) = 0;

   /**
    * Get the current end effector pose.
    * @return An Eigen::Transform<double, 3, Eigen::Affine> homogeneous transformation containing the pose of the end effector.
    * @warning This method should be called only after the kinematics has been updated via ForwardKinematics::update().
    */
    virtual Eigen::Transform<double, 3, Eigen::Affine> get_ee_transform() = 0;

   /**
    * Get the current end effector Jacobian \f$ J(q) \f$ such that
    * \f[
    * J(q) \dot{q} =
    * \begin{bmatrix}
    * v \\
    * \omega
    * \end{bmatrix}
    * \f]
    * with \f$ q \f$ the joints angles, \f$ v \f$ the linear velocity of the end-effector expressed in root frame and
    * \f$ \omega \f$ the angular velocity of the end-effector expressed in the root frame.
    * @return An Eigen::MatrixXd containing the Jacobian matrix.
    * @warning This method should be called only after the kinematics has been updated via ForwardKinematics::update().
    */
    virtual Eigen::MatrixXd get_jacobian() = 0;

    virtual Eigen::Vector3d get_ee_lin_vel() = 0;

    virtual Eigen::Vector3d get_ee_ang_vel() = 0;

    virtual Eigen::Vector3d get_ee_lin_acc() = 0;

    virtual Eigen::Vector3d get_ee_ang_acc() = 0;

   /**
    * Get the bias acceleration (i.e. acceleration not due to robot acceleration)
    * of the frame velocity. This term is usually called \f$ \dot{J(q)} \dot{q} \f$,
    * with \f$ q \f$ the joint velocities, \f$ J(q) \f$ the end effector Jacobian.
    * @return An Eigen::VectorXd containing the bias acceleration.
    * @warning This method should be called only after the kinematics has been updated via ForwardKinematics::update().
    */
    virtual Eigen::VectorXd get_ee_bias_acc() = 0;
};


#endif /* FORWARD_KINEMATICS_H */
