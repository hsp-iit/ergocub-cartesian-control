#ifndef POSITION_TRAJECTORY_GENERATOR_H
#define POSITION_TRAJECTORY_GENERATOR_H

#include <Eigen/Dense>
#include <memory>

#include <trajectory-generator/polynomial.h>


class PositionTrajectory
{
public:

    PositionTrajectory(const double sample_time);

    bool init(  const double duration,
                const Eigen::Vector3d& final_position,
                const Eigen::Vector3d& initial_position,
                const Eigen::Vector3d& initial_velocity = Eigen::Vector3d::Zero(),
                const Eigen::Vector3d& initial_acceleration = Eigen::Vector3d::Zero());

    bool setNewTarget(const double duration, const Eigen::Vector3d& final_position);

    void reset();

    bool isInitialized();

    bool isEnd();

    void step();

    Eigen::Vector3d getPosition();

    Eigen::Vector3d getVelocity();

    Eigen::Vector3d getAcceleration();

private:

    bool is_end_, is_init_;
    double t_, sample_time_, duration_;
    std::unique_ptr<Polynomial> polyx_, polyy_, polyz_;

    Eigen::Vector3d position_, velocity_, acceleration_;

    std::string class_name_ = "PositionTrajectory";
};


#endif
