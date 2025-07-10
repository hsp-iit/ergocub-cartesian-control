#include <trajectory-generator/positionTrajectoryGenerator.h>
#include <iostream>

PositionTrajectory::PositionTrajectory(const double sample_time)
{

    if(sample_time <= 0)
    {
        throw(std::runtime_error(class_name_ + "::ctor. sample_time <= 0."));
    }

    sample_time_ = sample_time;

    reset();
}

bool PositionTrajectory::init(  const double duration,
                                const Eigen::Vector3d& final_position,
                                const Eigen::Vector3d& initial_position,
                                const Eigen::Vector3d& initial_velocity,
                                const Eigen::Vector3d& initial_acceleration)
{
    if(duration<= 0)
    {
        std::cerr<<"[PositionTrajectory::init] Duration is negative!";
        return false;
    }

    duration_ = duration;
    t_ = 0.0;
    position_ = initial_position;
    velocity_ = initial_velocity;
    acceleration_ = initial_acceleration;

    // Calculate boundary conditions
    Eigen::VectorXd boundary_conditions_1 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd boundary_conditions_2 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd boundary_conditions_3 = Eigen::VectorXd::Zero(6);

    boundary_conditions_1(0) = initial_position.x();
    boundary_conditions_1(1) = initial_velocity.x();
    boundary_conditions_1(2) = initial_acceleration.x();
    boundary_conditions_1(3) = final_position.x();

    boundary_conditions_2(0) = initial_position.y();
    boundary_conditions_2(1) = initial_velocity.y();
    boundary_conditions_2(2) = initial_acceleration.y();
    boundary_conditions_2(3) = final_position.y();

    boundary_conditions_3(0) = initial_position.z();
    boundary_conditions_3(1) = initial_velocity.z();
    boundary_conditions_3(2) = initial_acceleration.z();
    boundary_conditions_3(3) = final_position.z();

    // Initialization
    polyx_ = std::make_unique<Polynomial>(boundary_conditions_1, duration);
    polyy_ = std::make_unique<Polynomial>(boundary_conditions_2, duration);
    polyz_ = std::make_unique<Polynomial>(boundary_conditions_3, duration);


    is_end_ = false;
    is_init_ = true;

    return true;
}


bool PositionTrajectory::setNewTarget(const double duration, const Eigen::Vector3d& final_position)
{
    return init(duration, final_position, position_, velocity_, acceleration_);
}


void PositionTrajectory::reset()
{
    is_end_ = true;
    is_init_ = false;

    t_ = 0.0;

    position_ = Eigen::Vector3d::Zero();
    velocity_ = Eigen::Vector3d::Zero();
    acceleration_ = Eigen::Vector3d::Zero();

    duration_ = 0.0;

    polyx_ = std::make_unique<Polynomial>(Eigen::VectorXd::Zero(6), 1.0);
    polyy_ = std::make_unique<Polynomial>(Eigen::VectorXd::Zero(6), 1.0);
    polyz_ = std::make_unique<Polynomial>(Eigen::VectorXd::Zero(6), 1.0);
}


bool PositionTrajectory::isInitialized()
{
    return is_init_;
}


void PositionTrajectory::step()
{
    /* Step time. */
    double t_new = t_ + sample_time_;
    if (t_new > duration_)
    {
        t_new = duration_;
        is_end_ = true;
    }

    t_ = t_new;

    // update orientation
    position_.x() = polyx_->at(t_);
    position_.y() = polyy_->at(t_);
    position_.z() = polyz_->at(t_);

    // update velocity
    velocity_.x() = polyx_->dt1At(t_);
    velocity_.y() = polyy_->dt1At(t_);
    velocity_.z() = polyz_->dt1At(t_);

    // update acceleration
    acceleration_.x() = polyx_->dt2At(t_);
    acceleration_.y() = polyy_->dt2At(t_);
    acceleration_.z() = polyz_->dt2At(t_);
}


bool PositionTrajectory::isEnd()
{
    return is_end_;
}

Eigen::Vector3d PositionTrajectory::getPosition()
{
    return position_;
}


Eigen::Vector3d PositionTrajectory::getVelocity()
{
    return velocity_;
}


Eigen::Vector3d PositionTrajectory::getAcceleration()
{
    return acceleration_;
}
