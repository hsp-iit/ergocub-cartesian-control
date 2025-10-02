#include <module.h>

#include <yarp/os/LogStream.h>
#include <yarp/eigen/Eigen.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool Module::configureService(const yarp::os::ResourceFinder& rf)
{
    std::string rpc_port_name = rf.check("rpc_port_name", yarp::os::Value("/" + module_name_ + "/rpc:i")).asString();

    if (!port_rpc_.open(rpc_port_name))
    {
        yError() << module_name_ + "::configureService(). Error: cannot open port" << rpc_port_name;
        return false;
    }

    if (!this->yarp().attachAsServer(port_rpc_))
    {
        yError() << module_name_ + "::configureService(). Error: cannot attach port" << rpc_port_name;
        return false;
    }

    return true;
}


bool Module::setOrientation(const yarp::sig::Matrix& rot)
{
    std::lock_guard<std::mutex> lg(mutex_);
    
    R_desired_ = yarp::eigen::toEigen(rot);

    state_ = Module::State::Running;

    return true;
}

bool Module::setOrientationFlat(double r11, double r12, double r13, double r21, double r22, double r23, double r31, double r32, double r33)
{
    yarp::sig::Matrix rot(3, 3);
    rot(0, 0) = r11; rot(0, 1) = r12; rot(0, 2) = r13;
    rot(1, 0) = r21; rot(1, 1) = r22; rot(1, 2) = r23;
    rot(2, 0) = r31; rot(2, 1) = r32; rot(2, 2) = r33;
    return setOrientation(rot);
}

bool Module::rotateAxisAngle(const double x, const double y, const double z, const double angle)
{
    std::lock_guard<std::mutex> lg(mutex_);
    
    R_desired_ = Eigen::AngleAxisd(angle * (M_PI/180.0), Eigen::Vector3d(x, y, z)).toRotationMatrix() * fk_.getTransform().rotation();

    state_ = Module::State::Running;

    return true;
}


bool Module::goHome()
{
    std::lock_guard<std::mutex> lg(mutex_);
    
    R_desired_ = Eigen::Matrix3d::Identity();

    state_ = Module::State::Running;

    return true;
}


bool Module::stop()
{
    std::lock_guard<std::mutex> lg(mutex_);

    state_ = Module::State::Idle;

    return true;
}

