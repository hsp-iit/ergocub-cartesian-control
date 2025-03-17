#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <Eigen/Dense>
#include <memory>

class Polynomial
{
public:
    Polynomial(const Eigen::VectorXd& boundary_conditions, const double length);

    //set interval [0,length]
    bool setIntervalLength(double length);
    //set initial and final conditions on the polynomial 
    bool setBoundaryConditions(const Eigen::VectorXd& boundary_conditions);

    //(quintic) polynomial evaluated at t
    double at(double t);
    //first order time derivative of the polynomial evaluated at t
    double dt1At(double t);
    //second order time derivative of the polynomial evaluated at t
    double dt2At(double t);

private:

    double length_;
    Eigen::VectorXd coefficients_;
    Eigen::VectorXd boundary_conditions_;

    //update polynomial coefficients
    void updateCoefficients();

    std::string class_name_ = "Polynomial";
};

#endif
