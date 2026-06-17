#include <ergocub_cartesian_bimanual/module.h>

#include <yarp/os/LogStream.h>
#include <yarp/eigen/Eigen.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool Module::go_to_position(double x, double y, double z, const std::string& arm)
{
    Eigen::Affine3d target_pose;
    target_pose = Eigen::Translation3d(Eigen::Vector3d(x, y, z));

    if(strcmp(arm.c_str(), "right")==0)
    {
        if (right_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Right arm is not enabled.";
            return false;
        }
        target_pose.rotate(right_desired_pose_.rotation());
        right_desired_pose_ = target_pose;
    }
    else if(strcmp(arm.c_str(), "left")==0)
    {
        if (left_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Left arm is not enabled.";
            return false;
        }
        target_pose.rotate(left_desired_pose_.rotation());
        left_desired_pose_ = target_pose;
    }
    else
    {
        yError() << "[" + module_name_ + "::" + __func__ + "] Error: invalid arm name.";
        return false;
    }

    setState(State::Running);
    return true;
}

bool Module::rotate_deg(double angle, double x, double y, double z, const std::string& arm)
{
    if(!rotate_rad(angle * (M_PI/180.0), x, y, z, arm))
    {
        yError() << "[" + module_name_ + "::" + __func__ + "] See error(s) above.";
        return false;
    }

    return true;
}

bool Module::rotate_rad(double angle, double x, double y, double z, const std::string& arm)
{
    if(strcmp(arm.c_str(), "right")==0)
    {
        if (right_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Right arm is not enabled.";
            return false;
        }
        right_desired_pose_.rotate(Eigen::AngleAxis(angle, Eigen::Vector3d(x,y,z)));
    }
    else if(strcmp(arm.c_str(), "left")==0)
    {
        if (left_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Left arm is not enabled.";
            return false;
        }
        left_desired_pose_.rotate(Eigen::AngleAxis(angle, Eigen::Vector3d(x,y,z)));
    }
    else
    {
        yError() << "[" + module_name_ + "::" + __func__ + "] Error: invalid arm name.";
        return false;
    }

    setState(State::Running);
    return true;
}

bool Module::go_home()
{
    if (right_enabled_)
    {
        right_desired_pose_ = right_chain_.home_pose;
    }
    if (left_enabled_)
    {
        left_desired_pose_ = left_chain_.home_pose;
    }

    setState(State::Running);
    return true;
}

bool Module::stop()
{
    setState(State::Stop);

    return true;
}

bool Module::checkAndReadQuery()
{
    yarp::os::Bottle* cmd = query_port_.read(false);

    if (cmd == nullptr || cmd->size() == 0)
    {
        return false;
    }

    const std::string query = cmd->get(0).asString();

    if      (query == "go_to_position")
    {
        if (cmd->size() == 5)
        {
            // Legacy format: go_to_position x y z arm
            go_to_position(cmd->get(1).asFloat64(), cmd->get(2).asFloat64(), cmd->get(3).asFloat64(),
                           cmd->get(4).asString());
            return true;
        }

        if (cmd->size() == 7)
        {
            // New format: go_to_position left(x y z) right(x y z)
            go_to_position(cmd->get(1).asFloat64(), cmd->get(2).asFloat64(), cmd->get(3).asFloat64(), "left");
            go_to_position(cmd->get(4).asFloat64(), cmd->get(5).asFloat64(), cmd->get(6).asFloat64(), "right");
            return true;
        }

        if (cmd->size() < 5)
        {
            yWarning() << "[" + module_name_ + "::" + __func__ + "] Expected 5 elements, got" << cmd->size();
            return true;
        }

        yWarning() << "[" + module_name_ + "::" + __func__ + "] Invalid size"
                   << cmd->size() << "(supported: 5 legacy, 7 bimanual left+right)";
        return true;
    }
    else if (query == "rotate_deg")
    {
        if (cmd->size() == 6)
        {
            // Legacy format: rotate_deg angle x y z arm
            rotate_deg(cmd->get(1).asFloat64(), cmd->get(2).asFloat64(), cmd->get(3).asFloat64(),
                       cmd->get(4).asFloat64(), cmd->get(5).asString());
            return true;
        }

        if (cmd->size() == 11)
        {
            // New format: rotate_deg left(angle x y z) right(angle x y z)
            rotate_deg(cmd->get(1).asFloat64(), cmd->get(2).asFloat64(), cmd->get(3).asFloat64(),
                       cmd->get(4).asFloat64(), "left");
            rotate_deg(cmd->get(5).asFloat64(), cmd->get(6).asFloat64(), cmd->get(7).asFloat64(),
                       cmd->get(8).asFloat64(), "right");
            return true;
        }

        if (cmd->size() < 6)
        {
            yWarning() << "[" + module_name_ + "::" + __func__ + "] Expected 6 elements, got" << cmd->size();
            return true;
        }

        yWarning() << "[" + module_name_ + "::" + __func__ + "] Invalid size"
                   << cmd->size() << "(supported: 6 legacy, 11 bimanual left+right)";
        return true;
    }
    else if (query == "rotate_rad")
    {
        if (cmd->size() == 6)
        {
            // Legacy format: rotate_rad angle x y z arm
            rotate_rad(cmd->get(1).asFloat64(), cmd->get(2).asFloat64(), cmd->get(3).asFloat64(),
                       cmd->get(4).asFloat64(), cmd->get(5).asString());
            return true;
        }

        if (cmd->size() == 11)
        {
            // New format: rotate_rad left(angle x y z) right(angle x y z)
            rotate_rad(cmd->get(1).asFloat64(), cmd->get(2).asFloat64(), cmd->get(3).asFloat64(),
                       cmd->get(4).asFloat64(), "left");
            rotate_rad(cmd->get(5).asFloat64(), cmd->get(6).asFloat64(), cmd->get(7).asFloat64(),
                       cmd->get(8).asFloat64(), "right");
            return true;
        }

        if (cmd->size() < 6)
        {
            yWarning() << "[" + module_name_ + "::" + __func__ + "] Expected 6 elements, got" << cmd->size();
            return true;
        }

        yWarning() << "[" + module_name_ + "::" + __func__ + "] Invalid size"
                   << cmd->size() << "(supported: 6 legacy, 11 bimanual left+right)";
        return true;
    }
    else if (query == "go_home")
    {
        go_home();
        return true;
    }
    else if (query == "stop")
    {
        stop();
        return true;
    }
    else
    {
        yWarning() << "[" + module_name_ + "::" + __func__ + "] Unknown command:" << query;
    }

    return true;
}
