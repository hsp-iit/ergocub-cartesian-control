// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include <trajectory-generator/orientationTrajectoryGenerator.h>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>



OrientationTrajectory::OrientationTrajectory(const double sample_time)
{
    is_end_ = true;

    if(sample_time <= 0)
    {
        throw(std::runtime_error(class_name_ + "::ctor. sample_time <= 0."));
    }

    sample_time_ = sample_time;

    reset();
}


bool OrientationTrajectory::init(   const double duration,
                                    const Eigen::Matrix3d& final_orientation,
                                    const Eigen::Matrix3d& initial_orientation,
                                    const Eigen::Vector3d& initial_velocity,
                                    const Eigen::Vector3d& initial_acceleration)
{
    if(duration<= 0)
    {
        std::cerr<<"[OrientationTrajectory::init] Duration is negative!";
        return false;
    }

    duration_ = duration;
    t_ = 0.0;
    orientation_ = initial_orientation;
    velocity_ = initial_velocity;
    acceleration_ = initial_acceleration;
    initial_orientation_ = initial_orientation;

    // Evaluate direction and magnitude of the initial_velocity
    const double init_v_norm = initial_velocity.norm();

    Eigen::Vector3d init_v_axis;
    if(init_v_norm == 0 )
        init_v_axis = Eigen::Vector3d::Zero();
    else
        init_v_axis = initial_velocity.normalized();

    // Evaluate direction and magnitude of the initial_acceleration
    const double init_a_norm = initial_acceleration.norm();

    Eigen::Vector3d init_a_axis;
    if(init_a_norm == 0 )
        init_a_axis = Eigen::Vector3d::Zero();
    else
        init_a_axis = initial_acceleration.normalized();

    // Evaluate direction and magnitude of the angular error
    double error_magnitude;
    Eigen::Vector3d error_axis;
    if(final_orientation == initial_orientation)
    {
        error_magnitude = 0;

        if (init_v_norm != 0)
            error_axis = init_v_axis;
        else if (init_a_norm != 0)
            error_axis = init_a_axis;
        else
            error_axis = Eigen::Vector3d::Zero();
    }
    else
    {
        Eigen::AngleAxisd rotation_error(final_orientation * initial_orientation.transpose());
        error_magnitude = rotation_error.angle();
        error_axis = rotation_error.axis();
    }

    // if the vectors are all parallel => it can generate a min jerk trajectory
    if(error_axis.cross(initial_velocity).isZero() && error_axis.cross(initial_acceleration).isZero())
    {
        //Create frame
        v1_ = error_axis;
        v2_ = Eigen::Vector3d::Zero();
        v3_ = Eigen::Vector3d::Zero();
    }
    else
    {
        //Create frame
        v1_ = error_axis;

        if(init_v_norm != 0)
            v2_ = v1_.cross(init_v_axis);
        else
            v2_ = v1_.cross(init_a_axis);

        v3_ = v1_.cross(v2_);
    }


    // Calculate boundary conditions
    Eigen::VectorXd boundary_conditions_1 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd boundary_conditions_2 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd boundary_conditions_3 = Eigen::VectorXd::Zero(6);

    boundary_conditions_1(1) = initial_velocity.dot(v1_);
    boundary_conditions_1(2) = initial_acceleration.dot(v1_);
    boundary_conditions_1(3) = error_magnitude;

    boundary_conditions_2(1) = initial_velocity.dot(v2_);
    boundary_conditions_2(2) = initial_acceleration.dot(v2_);

    boundary_conditions_3(1) = initial_velocity.dot(v3_);
    boundary_conditions_3(2) = initial_acceleration.dot(v3_);

    // Initialization
    poly1_ = std::make_unique<Polynomial>(boundary_conditions_1, duration);
    poly2_ = std::make_unique<Polynomial>(boundary_conditions_2, duration);
    poly3_ = std::make_unique<Polynomial>(boundary_conditions_3, duration);


    is_end_ = false;
    is_init_ = true;

    return true;
}


bool OrientationTrajectory::setNewTarget(const double duration, const Eigen::Matrix3d& final_orientation)
{
    return init(duration, final_orientation, orientation_, velocity_, acceleration_);
}


bool OrientationTrajectory::isInitialized()
{
    return is_init_;
}


void OrientationTrajectory::reset()
{
    is_end_ = true;
    is_init_ = false;

    t_ = 0.0;

    orientation_ = Eigen::Matrix3d::Zero();
    velocity_ = Eigen::Vector3d::Zero();
    acceleration_ = Eigen::Vector3d::Zero();
    initial_orientation_ = Eigen::Matrix3d::Zero();

    duration_ = 0.0;

    poly1_ = std::make_unique<Polynomial>(Eigen::VectorXd::Zero(6), 1.0);
    poly2_ = std::make_unique<Polynomial>(Eigen::VectorXd::Zero(6), 1.0);
    poly3_ = std::make_unique<Polynomial>(Eigen::VectorXd::Zero(6), 1.0);
}


void OrientationTrajectory::step()
{
    /* Step time. */
    double t_new = t_ + sample_time_;
    if (t_new > duration_)
    {
        t_new = duration_;
        is_end_ = true;
    }

    t_ = t_new;

    /*
        Notice that here and in [0] the trajectory is generated w.r.t. the robot's root frame,
        while in [1] and [2] the formula is expressed w.r.t. the end-effector frame.

        References:
        [0] https://github.com/hsp-iit/grasping-baselines/issues/90#issuecomment-2033945717
        [1] https://www.researchgate.net/publication/3298848_On_the_Generation_of_Smooth_Three-Dimensional_Rigid_Body_Motions
        [2] https://ami-iit.github.io/bipedal-locomotion-framework/so3-minjerk.html
    */

    /* Helper function. */
    auto skewOperator  = [](const Eigen::Vector3d& vec)
    {
        Eigen::Matrix3d S;

        S(0, 0) = 0;        S(0, 1) = -vec.z(); S(0, 2) = vec.y();
        S(1, 0) = vec.z();  S(1, 1) = 0;        S(1, 2) = -vec.x();
        S(2, 0) = -vec.y(); S(2, 1) = vec.x();  S(2, 2) = 0;

        return S;
    };

    // update orientation
    const Eigen::Vector3d angle_axis = poly1_->at(t_) * v1_ + poly2_->at(t_) * v2_ + poly3_->at(t_) * v3_;
    orientation_ = skewOperator(angle_axis).exp() * initial_orientation_;

    // update velocity
    velocity_ = poly1_->dt1At(t_) * v1_ + poly2_->dt1At(t_) * v2_ + poly3_->dt1At(t_) * v3_;

    // update acceleration
    acceleration_ = poly1_->dt2At(t_) * v1_ + poly2_->dt2At(t_) * v2_ + poly3_->dt2At(t_) * v3_;
}


Eigen::Matrix3d OrientationTrajectory::getOrientation()
{
    return orientation_;
}


Eigen::Vector3d OrientationTrajectory::getVelocity()
{
    return velocity_;
}


Eigen::Vector3d OrientationTrajectory::getAcceleration()
{
    return acceleration_;
}


bool OrientationTrajectory::isEnd()
{
    return is_end_;
}
