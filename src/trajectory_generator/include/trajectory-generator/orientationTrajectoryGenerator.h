#ifndef ORIENTATION_TRAJECTORY_GENERATOR_H
#define ORIENTATION_TRAJECTORY_GENERATOR_H

// This algorithm generates a minimum jerk trajectory only if the necessary
// conditions are satisfied (see On the Generation of Smooth Three-Dimensional
// Rigid Body Motions - Miloš Žefran, Vijay Kumar, and Christopher B.
// Croke), otherwise it generates a trajectory that, even if it is the composition
// of min jerk trajectories, is NOT min jerk!

#include <Eigen/Dense>
#include <memory>

#include <trajectory-generator/polynomial.h>


class OrientationTrajectory
{
public:

    OrientationTrajectory(const double sample_time);

    bool init(  const double duration,
                const Eigen::Matrix3d& final_orientation,
                const Eigen::Matrix3d& initial_orientation,
                const Eigen::Vector3d& initial_velocity = Eigen::Vector3d::Zero(),
                const Eigen::Vector3d& initial_acceleration = Eigen::Vector3d::Zero());

    bool setNewTarget(const double duration, const Eigen::Matrix3d& final_orientation);

    void reset();

    bool isInitialized();

    bool isEnd();

    void step();

    Eigen::Matrix3d getOrientation();

    Eigen::Vector3d getVelocity();

    Eigen::Vector3d getAcceleration();

private:

    bool is_end_, is_init_;
    double t_, sample_time_, duration_;
    std::unique_ptr<Polynomial> poly1_, poly2_, poly3_;

    Eigen::Matrix3d orientation_, initial_orientation_;
    Eigen::Vector3d v1_, v2_, v3_, velocity_, acceleration_;

    std::string class_name_ = "OrientationTrajectory";
};

#endif
