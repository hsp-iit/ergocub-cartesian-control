#include <module.h>

#include <yarp/os/LogStream.h>
#include <yarp/eigen/Eigen.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool Module::configureService(const yarp::os::ResourceFinder &rf, const std::string rpc_port_name)
{
    if (!rpc_cmd_port_.open(rpc_port_name))
    {
        yError()<< "[" + module_name_ + "::configureService] Error: cannot open port" << rpc_port_name;
        return false;
    }

    if (!attach(rpc_cmd_port_))
    {
        yError() << "[" + module_name_ + "::configureService] Error: cannot attach port" << rpc_port_name;
        return false;
    }

    return true;
}

bool Module::attach(yarp::os::Port &source)
{
    return this->yarp().attachAsServer(source);
}

bool Module::flat_go_to_pose(double x, double y, double z, double m1, double m2, double m3, double m4, double m5, double m6, double m7, double m8, double m9, const std::string& arm)
{    

    Eigen::Affine3d target_pose;
    target_pose = Eigen::Translation3d(x, y, z);
    Eigen::Matrix3d rotation;
    rotation << m1, m2, m3,
                m4, m5, m6,
                m7, m8, m9;
    target_pose.rotate(rotation);

    if(strcmp(arm.c_str(), "right")==0)
    {
        if (right_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::flat_go_to_pose] Right arm is not enabled.";
            return false;
        }
        right_desired_pose_ = target_pose;
    }
    else if(strcmp(arm.c_str(), "left")==0)
    {
        if (left_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::flat_go_to_pose] Left arm is not enabled.";
            return false;
        }
        left_desired_pose_ = target_pose;
    }
    else
    {
        yError() << "[" + module_name_ + "::flat_go_to_pose] Error: invalid arm name.";
        return false;
    }

    setState(State::Running);
    return true;
}

bool Module::go_to_pose(double x, double y, double z, double q_x, double q_y, double q_z, double q_w, const std::string& arm)
{    

    Eigen::Affine3d target_pose;
    target_pose = Eigen::Translation3d(x, y, z);
    target_pose.rotate(Eigen::Quaterniond(q_w, q_x, q_y, q_z));

    if(strcmp(arm.c_str(), "right")==0)
    {
        if (right_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::go_to_pose] Right arm is not enabled.";
            return false;
        }
        right_desired_pose_ = target_pose;
    }
    else if(strcmp(arm.c_str(), "left")==0)
    {
        if (left_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::go_to_pose] Left arm is not enabled.";
            return false;
        }
        left_desired_pose_ = target_pose;
    }
    else
    {
        yError() << "[" + module_name_ + "::go_to_pose] Error: invalid arm name.";
        return false;
    }

    setState(State::Running);
    return true;
}


bool Module::go_to_position(double x, double y, double z, const std::string& arm)
{
    Eigen::Affine3d target_pose;
    target_pose = Eigen::Translation3d(Eigen::Vector3d(x, y, z));

    yInfo() << "[" + module_name_ + "::go_to_position] arm.c_str() "<<arm.c_str();
    yInfo() << "[" + module_name_ + "::go_to_position] strcmp(arm.c_str(), right) "<<strcmp(arm.c_str(), "right");
    yInfo() << "[" + module_name_ + "::go_to_position] strcmp(arm.c_str(), left) "<<strcmp(arm.c_str(), "left");

    if(strcmp(arm.c_str(), "right")==0)
    {
        if (right_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::go_to_position] Right arm is not enabled.";
            return false;
        }
        target_pose.rotate(right_desired_pose_.rotation());
        right_desired_pose_ = target_pose;
    }
    else if(strcmp(arm.c_str(), "left")==0)
    {
        if (left_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::go_to_position] Left arm is not enabled.";
            return false;
        }
        target_pose.rotate(left_desired_pose_.rotation());
        left_desired_pose_ = target_pose;
    }
    else
    {
        yError() << "[" + module_name_ + "::go_to_position] Error: invalid arm name.";
        return false;
    }

    setState(State::Running);
    return true;
}

bool Module::rotate_deg(double angle, double x, double y, double z, const std::string& arm)
{
    if(!rotate_rad(angle * (M_PI/180.0), x, y, z, arm))
    {
        yError() << "[" + module_name_ + "::rotate_deg] See error(s) above.";
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
            yError() << "[" + module_name_ + "::rotate_rad] Right arm is not enabled.";
            return false;
        }
        right_desired_pose_.rotate(Eigen::AngleAxis(angle, Eigen::Vector3d(x,y,z)));
    }
    else if(strcmp(arm.c_str(), "left")==0)
    {
        if (left_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::rotate_rad] Left arm is not enabled.";
            return false;
        }
        left_desired_pose_.rotate(Eigen::AngleAxis(angle, Eigen::Vector3d(x,y,z)));
    }
    else
    {
        yError() << "[" + module_name_ + "::rotate_rad] Error: invalid arm name.";
        return false;
    }

    setState(State::Running);
    return true;
}


yarp::sig::Matrix Module::get_pose(const std::string& arm)
{
    yarp::sig::Matrix yarp_pose;

    yarp_pose.resize(4, 4);

    if(strcmp(arm.c_str(), "right")==0)
    {
        if (right_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::getPose(). Right arm is not enabled.";
            return yarp_pose;
        }
        yarp::eigen::toEigen(yarp_pose) = right_chain_.measFk->get_ee_transform().matrix();
    }
    else if(strcmp(arm.c_str(), "left")==0)
    {
        if (left_enabled_ == false)
        {
            yError() << "[" + module_name_ + "::getPose(). Left arm is not enabled.";
            return yarp_pose;
        }
        yarp::eigen::toEigen(yarp_pose) = left_chain_.measFk->get_ee_transform().matrix();
    }
    else
    {
        yError() << "[" + module_name_ + "::rotate_rad] Error: invalid arm name.";
        yarp::eigen::toEigen(yarp_pose) = -1 * Eigen::Matrix4d::Identity();
    }

    return yarp_pose;
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
    // right_desired_pose_ = right_chain_.home_pose;
    // left_desired_pose_ = left_chain_.home_pose;

    setState(State::Running);
    return true;
}

bool Module::stop()
{
    setState(State::Stop);

    return true;
}

