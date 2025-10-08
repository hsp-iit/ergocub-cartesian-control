#ifndef INVERSE_KINEMATICS_PARAMETERS_H
#define INVERSE_KINEMATICS_PARAMETERS_H

#include <yarp/os/Bottle.h>

#include <vector>
#include <string>


class InverseKinematicsParameters
{
public:
    bool configure(const yarp::os::Bottle& parameters);

    double p_gain() const;

    double weight_vel() const;

    double gain_limits() const;

private:
    double p_gain_, weight_vel_, gain_limits_;
};

#endif /* INVERSE_KINEMATICS_PARAMETERS_H */