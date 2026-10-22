#include <inverseKinematicsParameters.h>

#include <yarp/os/LogStream.h>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


bool InverseKinematicsParameters::configure(const yarp::os::Bottle& parameters)
{
    const std::string class_name = "InverseKinematicsParameters";

    const std::vector<std::string> parameters_names = \
    {
        "p_gain",
        "weight_vel",
        "gain_limits",
        "max_vel"
    };

    /* Check parameters and validity. */
    bool valid = true;
    for (const auto& key : parameters_names)
    {
        if (!parameters.check(key) || !parameters.find(key).isFloat64() || parameters.find(key).asFloat64() < 0)
        {
            valid = false;
            yError() << class_name + "::configure(). Error: missing, negative or invalid " + key + " parameter.";
        }
    }

    /* Assign parameters. */
    p_gain_ = parameters.find("p_gain").asFloat64();
    weight_vel_ = parameters.find("weight_vel").asFloat64();
    gain_limits_ = parameters.find("gain_limits").asFloat64();
    max_vel_ = parameters.find("max_vel").asFloat64() * M_PI / 180.0;

    /* Print values. */
    yDebug() << class_name + "::configure(). p_gain: " + std::to_string(p_gain_);
    yDebug() << class_name + "::configure(). weight_vel: " + std::to_string(weight_vel_);
    yDebug() << class_name + "::configure(). gain_limits: " + std::to_string(gain_limits_);
    yDebug() << class_name + "::configure(). max_vel [rad/s]: " + std::to_string(max_vel_);

    return valid;
}


double InverseKinematicsParameters::p_gain() const
{
    return p_gain_;
}


double InverseKinematicsParameters::weight_vel() const
{
    return weight_vel_;
}

double InverseKinematicsParameters::gain_limits() const
{
    return gain_limits_;
}

double InverseKinematicsParameters::max_vel() const
{
    return max_vel_;
}
