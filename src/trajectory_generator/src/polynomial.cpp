// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include <trajectory-generator/polynomial.h>
#include <iostream>


Polynomial::Polynomial(const Eigen::VectorXd& boundary_conditions, const double length)
{
    length_= 0.0;
    coefficients_ = Eigen::VectorXd::Zero(6);
    boundary_conditions_ = Eigen::VectorXd::Zero(6);

    if(!setIntervalLength(length))
    {
        throw(std::runtime_error(class_name_ + "::ctor. See error(s) above."));
    }

    if(!setBoundaryConditions(boundary_conditions))
    {
        throw(std::runtime_error(class_name_ + "::ctor. See error(s) above."));
    }

}


bool Polynomial::setIntervalLength(double length)
{
    if (length <= 0)
    {
        std::cerr << class_name_ + "::setIntervalLength(). Error: length <= 0.";
        return false;
    }

    length_ = length;

    updateCoefficients();

    return true;
}


bool Polynomial::setBoundaryConditions(const Eigen::VectorXd& boundary_conditions)
{
    if (boundary_conditions.size() != 6)
    {
        std::cerr << class_name_ + "::setBoundaryConditions(). Error: wrong size of boundary_conditions.";
        return false;
    }

    boundary_conditions_ = boundary_conditions;

    updateCoefficients();

    return true;
}


double Polynomial::at(double t)
{
    return  (coefficients_(0) +
            coefficients_(1) * t +
            coefficients_(2) * std::pow(t,2) +
            coefficients_(3) * std::pow(t,3) +
            coefficients_(4) * std::pow(t,4) +
            coefficients_(5) * std::pow(t,5));
}


double Polynomial::dt1At(double t)
{
    return  (coefficients_(1) +
            2 * coefficients_(2) * t +
            3 * coefficients_(3) * std::pow(t,2) +
            4 * coefficients_(4) * std::pow(t,3) +
            5 * coefficients_(5) * std::pow(t,4));
}


double Polynomial::dt2At(double t)
{
    return  (2 * coefficients_(2) +
            6 * coefficients_(3) * t +
            12 * coefficients_(4) * std::pow(t,2) +
            20 * coefficients_(5) * std::pow(t,3));
}


void Polynomial::updateCoefficients()
{
    /*
    boundary_conditions_(0) = s0
    boundary_conditions_(1) = ds0
    boundary_conditions_(2) = dds0
    boundary_conditions_(3) = sf
    boundary_conditions_(4) = dsf
    boundary_conditions_(5) = ddsf
    */

    coefficients_(0) = boundary_conditions_(0);
    coefficients_(1) = boundary_conditions_(1);
    coefficients_(2) = boundary_conditions_(2)/2;
    coefficients_(3) = 10 * (boundary_conditions_(3) - boundary_conditions_(0)) / std::pow(length_,3) - (6*boundary_conditions_(1) + 4*boundary_conditions_(4)) / std::pow(length_,2) - (3*boundary_conditions_(2) -   boundary_conditions_(5)) / (2*std::pow(length_,1));
    coefficients_(4) = 15 * (boundary_conditions_(0) - boundary_conditions_(3)) / std::pow(length_,4) + (8*boundary_conditions_(1) + 7*boundary_conditions_(4)) / std::pow(length_,3) + (3*boundary_conditions_(2) - 2*boundary_conditions_(5)) / (2*std::pow(length_,2));
    coefficients_(5) =  6 * (boundary_conditions_(3) - boundary_conditions_(0)) / std::pow(length_,5) - (3*boundary_conditions_(1) + 3*boundary_conditions_(4)) / std::pow(length_,4) - (  boundary_conditions_(2) -   boundary_conditions_(5)) / (2*std::pow(length_,3));
}
