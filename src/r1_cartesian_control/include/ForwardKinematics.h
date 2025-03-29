/*
 * Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * MIT license. See the accompanying LICENSE file for details.
 */

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
    * @param joints A Eigen::VectorXd containing the joint values in radian.
    */
    virtual void set_joints_state(const Eigen::Ref<const Eigen::VectorXd>& joints) = 0;

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
};


#endif /* FORWARD_KINEMATICS_H */
