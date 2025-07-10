/*
 * Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * MIT license. See the accompanying LICENSE file for details.
 */

#include <ForwardKinematicsiDynTree.h>

#include <iDynTree/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>

using namespace Eigen;
using namespace iDynTree;


ForwardKinematicsiDynTree::ForwardKinematicsiDynTree(const std::string& urdf_path, const std::vector<std::string>& joints_list, const std::string& root_frame_name, const std::string& ee_frame_name)
{
    /* Load the model. */
    ModelLoader loader;
    if (!loader.loadReducedModelFromFile(urdf_path, joints_list))
    {
        throw(std::runtime_error(log_prefix_ + "::ctor. Cannot load model from " + urdf_path));
    }

    /* Instantiate the chain. */
    chain_.loadRobotModel(loader.model());

    /* Check that the frames of interest do exist. */
    root_frame_idx_ = chain_.getFrameIndex(root_frame_name);
    ee_frame_idx_ = chain_.getFrameIndex(ee_frame_name);

    if (root_frame_idx_ < 0)
    {
        throw(std::runtime_error(log_prefix_ + "::ctor. Cannot find root frame " + root_frame_name + " in model " + urdf_path));
    }
    if (ee_frame_idx_ < 0)
    {
        throw(std::runtime_error(log_prefix_ + "::ctor. Cannot find end-effector frame " + ee_frame_name + " in model " + urdf_path));
    }

    /* Set the mixed twist representation.
       It means that:
       - the linear velocity is that of the origin of the end-effector frame expressed in the root frame;
       - the angular velocity is that of the end-effector frame frame expressed in the root frame.
    */
    chain_.setFrameVelocityRepresentation(FrameVelocityRepresentation::MIXED_REPRESENTATION);

    /* Resize the jacobian. */
    jacobian_.resize(6, chain_.getNrOfDegreesOfFreedom());

    joint_acc_.resize(chain_.getNrOfDegreesOfFreedom());

    lin_vel_.setZero();
    ang_vel_.setZero();
    lin_acc_.setZero();
    ang_acc_.setZero();

    bias_acc_.resize(6);
    bias_acc_ = Eigen::VectorXd::Zero(6);
}


void ForwardKinematicsiDynTree::update()
{
    /* End-effector transform. */
    //chain_.getWorldTransform(ee_frame_idx_, transform_.matrix());
    chain_.getRelativeTransform(root_frame_idx_, ee_frame_idx_, transform_.matrix());

    /* Jacobian. */
    chain_.getRelativeJacobian(root_frame_idx_, ee_frame_idx_, jacobian_);

    /* End-effector Velocity. */
    auto twist = toEigen(chain_.getFrameVel(ee_frame_idx_));
    lin_vel_ = twist.topRows(3);
    ang_vel_ = twist.bottomRows(3);

    /* End-effector Acceleration. */
    Vector6 baseAcc;
    baseAcc.zero();
    VectorDynSize jointAcc;
    jointAcc.resize(joint_acc_.size());
    toEigen(jointAcc) = joint_acc_;
    auto acc = chain_.getFrameAcc(ee_frame_idx_, baseAcc, jointAcc);
    lin_acc_ = toEigen(acc).topRows(3);
    ang_acc_ = toEigen(acc).bottomRows(3);

    /* Get the bias acceleration.  */
    bias_acc_ = toEigen(chain_.getFrameBiasAcc(ee_frame_idx_));
}


void ForwardKinematicsiDynTree::set_joints_state(const Ref<const VectorXd>& joints_pos, const Ref<const VectorXd>& joints_vel, const Ref<const VectorXd>& joints_acc)
{
    Vector3d gravity(0.0, 0.0, -9.81);

    chain_.setRobotState(joints_pos, joints_vel, gravity);

    joint_acc_ = joints_acc;
}


Eigen::Transform<double, 3, Affine> ForwardKinematicsiDynTree::get_ee_transform()
{
    return transform_;
}


MatrixXd ForwardKinematicsiDynTree::get_jacobian()
{
    return jacobian_;
}


Eigen::Vector3d ForwardKinematicsiDynTree::get_ee_lin_vel()
{
    return lin_vel_;
}


Eigen::Vector3d ForwardKinematicsiDynTree::get_ee_ang_vel()
{
    return ang_vel_;
}


Eigen::Vector3d ForwardKinematicsiDynTree::get_ee_lin_acc()
{
    return lin_acc_;
}


Eigen::Vector3d ForwardKinematicsiDynTree::get_ee_ang_acc()
{
    return ang_acc_;
}


Eigen::VectorXd ForwardKinematicsiDynTree::get_ee_bias_acc()
{
    return bias_acc_;
}
