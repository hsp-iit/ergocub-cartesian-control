#include <inverseKinematicsParameters.h>

#include <yarp/os/LogStream.h>


bool InverseKinematicsParameters::configure(const yarp::os::Bottle& parameters)
{
    const std::string class_name = "InverseKinematicsParameters";

    const std::vector<std::string> parameters_names = \
    {
        "p_gain",
        "weight_vel",
        "gain_limits"
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

    /* Print values. */
    yDebug() << class_name + "::configure(). p_gain: " + std::to_string(p_gain_);
    yDebug() << class_name + "::configure(). weight_vel: " + std::to_string(weight_vel_);
    yDebug() << class_name + "::configure(). gain_limits: " + std::to_string(gain_limits_);

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
