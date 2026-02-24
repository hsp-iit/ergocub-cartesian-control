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

    if (!rpc_cmd_port_.open(rpc_port_name))
    {
        yError() << module_name_ + "::configureService(). Error: cannot open port" << rpc_port_name;
        return false;
    }

    return true;
}

bool Module::checkAndReadRpcCommands()
{
    yarp::os::Bottle* cmd = rpc_cmd_port_.read(false);
    if (cmd == nullptr)
    {
        return false;
    }

    if (cmd->size() == 0)
    {
        return true;
    }

    const std::string op = cmd->get(0).asString();

    if (op == "setOrientationFlat" || op == "set_orientation_flat")
    {
        if (cmd->size() < 10)
        {
            yWarning() << module_name_ + "::checkAndReadRpcCommands(). setOrientationFlat expects 10 elements, got" << cmd->size();
            return true;
        }

        setOrientationFlat(cmd->get(1).asFloat64(), cmd->get(2).asFloat64(), cmd->get(3).asFloat64(),
                           cmd->get(4).asFloat64(), cmd->get(5).asFloat64(), cmd->get(6).asFloat64(),
                           cmd->get(7).asFloat64(), cmd->get(8).asFloat64(), cmd->get(9).asFloat64());
        return true;
    }

    if (op == "rotateAxisAngle" || op == "rotate_axis_angle")
    {
        if (cmd->size() < 5)
        {
            yWarning() << module_name_ + "::checkAndReadRpcCommands(). rotateAxisAngle expects 5 elements, got" << cmd->size();
            return true;
        }

        rotateAxisAngle(cmd->get(1).asFloat64(), cmd->get(2).asFloat64(), cmd->get(3).asFloat64(), cmd->get(4).asFloat64());
        return true;
    }

    if (op == "goHome" || op == "go_home")
    {
        goHome();
        return true;
    }

    if (op == "stop")
    {
        stop();
        return true;
    }

    yWarning() << module_name_ + "::checkAndReadRpcCommands(). Unknown command:" << op;
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

