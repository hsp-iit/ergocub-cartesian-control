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

bool Module::checkAndReadRpcCommands()
{
    yInfo() << "[" + module_name_ + "::checkAndReadRpcCommands] Checking for RPC commands...";
    yarp::os::Bottle* cmd = rpc_cmd_port_.read(false);
    // print received message
    if (cmd != nullptr)
    {
        yInfo() << "[" + module_name_ + "::checkAndReadRpcCommands] Received command:" << cmd->toString();
    }
    else
    {
        yInfo() << "[" + module_name_ + "::checkAndReadRpcCommands] No command received.";
    }
    if (cmd == nullptr)
    {
        return false;
    }

    if (cmd->size() == 0)
    {
        return true;
    }

    const std::string op = cmd->get(0).asString();

    if (op == "flat_go_to_pose")
    {
        if (cmd->size() == 25)
        {
            // New format: flat_go_to_pose left(x y z m1..m9) right(x y z m1..m9)
            flat_go_to_pose(cmd->get(1).asFloat64(), cmd->get(2).asFloat64(), cmd->get(3).asFloat64(),
                            cmd->get(4).asFloat64(), cmd->get(5).asFloat64(), cmd->get(6).asFloat64(),
                            cmd->get(7).asFloat64(), cmd->get(8).asFloat64(), cmd->get(9).asFloat64(),
                            cmd->get(10).asFloat64(), cmd->get(11).asFloat64(), cmd->get(12).asFloat64(),
                            "left");
            flat_go_to_pose(cmd->get(13).asFloat64(), cmd->get(14).asFloat64(), cmd->get(15).asFloat64(),
                            cmd->get(16).asFloat64(), cmd->get(17).asFloat64(), cmd->get(18).asFloat64(),
                            cmd->get(19).asFloat64(), cmd->get(20).asFloat64(), cmd->get(21).asFloat64(),
                            cmd->get(22).asFloat64(), cmd->get(23).asFloat64(), cmd->get(24).asFloat64(),
                            "right");
            return true;
        }

        else if (cmd->size() < 14)
        {
            yWarning() << "[" + module_name_ + "::checkAndReadRpcCommands] flat_go_to_pose expects 25 elements, got" << cmd->size();
            return true;
        }

        yWarning() << "[" + module_name_ + "::checkAndReadRpcCommands] flat_go_to_pose invalid size"
                   << cmd->size() << "(supported: 14 legacy, 25 bimanual left+right)";
        return true;
    }

    if (op == "go_to_pose")
    {
        if (cmd->size() == 9)
        {
            // Legacy format: go_to_pose x y z qx qy qz qw arm
            go_to_pose(cmd->get(1).asFloat64(), cmd->get(2).asFloat64(), cmd->get(3).asFloat64(),
                       cmd->get(4).asFloat64(), cmd->get(5).asFloat64(), cmd->get(6).asFloat64(),
                       cmd->get(7).asFloat64(), cmd->get(8).asString());
            return true;
        }

        if (cmd->size() == 15)
        {
            // New format: go_to_pose left(x y z qx qy qz qw) right(x y z qx qy qz qw)
            go_to_pose(cmd->get(1).asFloat64(), cmd->get(2).asFloat64(), cmd->get(3).asFloat64(),
                       cmd->get(4).asFloat64(), cmd->get(5).asFloat64(), cmd->get(6).asFloat64(),
                       cmd->get(7).asFloat64(), "left");
            go_to_pose(cmd->get(8).asFloat64(), cmd->get(9).asFloat64(), cmd->get(10).asFloat64(),
                       cmd->get(11).asFloat64(), cmd->get(12).asFloat64(), cmd->get(13).asFloat64(),
                       cmd->get(14).asFloat64(), "right");
            return true;
        }

        if (cmd->size() < 9)
        {
            yWarning() << "[" + module_name_ + "::checkAndReadRpcCommands] go_to_pose expects 9 elements, got" << cmd->size();
            return true;
        }

        yWarning() << "[" + module_name_ + "::checkAndReadRpcCommands] go_to_pose invalid size"
                   << cmd->size() << "(supported: 9 legacy, 15 bimanual left+right)";
        return true;
    }

    if (op == "go_to_position")
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
            yWarning() << "[" + module_name_ + "::checkAndReadRpcCommands] go_to_position expects 5 elements, got" << cmd->size();
            return true;
        }

        yWarning() << "[" + module_name_ + "::checkAndReadRpcCommands] go_to_position invalid size"
                   << cmd->size() << "(supported: 5 legacy, 7 bimanual left+right)";
        return true;
    }

    if (op == "rotate_deg")
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
            yWarning() << "[" + module_name_ + "::checkAndReadRpcCommands] rotate_deg expects 6 elements, got" << cmd->size();
            return true;
        }

        yWarning() << "[" + module_name_ + "::checkAndReadRpcCommands] rotate_deg invalid size"
                   << cmd->size() << "(supported: 6 legacy, 11 bimanual left+right)";
        return true;
    }

    if (op == "rotate_rad")
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
            yWarning() << "[" + module_name_ + "::checkAndReadRpcCommands] rotate_rad expects 6 elements, got" << cmd->size();
            return true;
        }

        yWarning() << "[" + module_name_ + "::checkAndReadRpcCommands] rotate_rad invalid size"
                   << cmd->size() << "(supported: 6 legacy, 11 bimanual left+right)";
        return true;
    }

    if (op == "go_home")
    {
        go_home();
        return true;
    }

    if (op == "stop")
    {
        stop();
        return true;
    }

    yWarning() << "[" + module_name_ + "::checkAndReadRpcCommands] Unknown command:" << op;
    return true;
}

