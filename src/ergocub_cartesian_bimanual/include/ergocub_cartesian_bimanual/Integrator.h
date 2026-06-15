/*
 * Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * MIT license. See the accompanying LICENSE file for details.
 */

#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <Eigen/Dense>

/**
 * Class implementing component-wise integration on an Eigen::VectorXd vector.
 */
class Integrator
{
public:
    /**
     * @brief Constructor.
     *
     * @param sampling_time The sampling time of the integrator.
     */
    Integrator(const double &sampling_time);

    /**
     * @brief Destructor.
     */
    ~Integrator() = default;

    /**
     * @brief Set the initial condition of the integrator.
     *
     * @param initial_state The initial state as a Eigen::VectorXd vector.
     */
    void set_initial_condition(const Eigen::Ref<const Eigen::VectorXd> &initial_state);

    /**
     * @brief Integrate the provided state derivative.
     *
     * @param state_derivative The state derivative as a Eigen::VectorXd vector.
     */
    void integrate(const Eigen::Ref<const Eigen::VectorXd> &state_derivative);

    /**
     * @brief Get the current state of the integrator.
     *
     * @return The current state as a Eigen::VectorXd vector
     */
    Eigen::VectorXd get_state();

private:
    double sampling_time_;

    Eigen::VectorXd state_;
};

#endif /* INTEGRATOR_H */