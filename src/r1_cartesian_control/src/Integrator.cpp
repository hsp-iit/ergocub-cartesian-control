// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include <Integrator.h>

#include <yarp/os/LogStream.h>

using namespace Eigen;

Integrator::Integrator(const double &sampling_time) : sampling_time_(sampling_time)
{
}

void Integrator::set_initial_condition(const Ref<const VectorXd> &initial_state)
{
    state_ = initial_state;
}

void Integrator::integrate(const Ref<const VectorXd> &state_derivative)
{
    if (state_.size() != state_derivative.size())
    {
        yError() << "Integrator::integrate(). Error: state_derivative.size() should be equal to " + std::to_string(state_.size())+ ".";
        return;
    }

    state_ += state_derivative * sampling_time_;
}

VectorXd Integrator::get_state()
{
    return state_;
}
