// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include <module.h>

#include <yarp/os/LogStream.h>
#include <yarp/eigen/Eigen.h>

bool Module::configureService(const yarp::os::ResourceFinder &rf, const std::string rpc_port_name)
{

    if (!rpc_cmd_port_.open(rpc_port_name))
    {
        yError() << module_name_ + "::configureService(). Error: cannot open port" << rpc_port_name;
        return false;
    }

    if (!attach(rpc_cmd_port_))
    {
        yError() << module_name_ + "::configureService(). Error: cannot attach port" << rpc_port_name;
        return false;
    }

    return true;
}

bool Module::attach(yarp::os::Port &source)
{
    return this->yarp().attachAsServer(source);
}

bool Module::go_to_pose(double x, double y, double z, double q_x, double q_y, double q_z, double q_w, double traj_duration)
{
    Eigen::Affine3d target_pose;
    target_pose = Eigen::Translation3d(x, y, z);
    target_pose.rotate(Eigen::Quaterniond(q_w, q_x, q_y, q_z));

    serviceTrajInit(false, target_pose, traj_duration);

    return true;
}


bool Module::go_to_position(double x, double y, double z, double traj_duration)
{
    Eigen::Affine3d target_pose;
    target_pose = Eigen::Translation3d(Eigen::Vector3d(x, y, z));
    target_pose.rotate(desired_transform_.rotation());

    serviceTrajInit(false, target_pose, traj_duration);

    return true;
}

bool Module::rotate_deg(double angle, double x, double y, double z, double traj_duration)
{
    return rotate_rad(angle * (M_PI/180.0), x, y, z, traj_duration);
}

bool Module::rotate_rad(double angle, double x, double y, double z, double traj_duration)
{

    Eigen::Affine3d target_pose;
    target_pose = getTrajFinPose();

    target_pose.rotate(Eigen::AngleAxis(angle, Eigen::Vector3d(x,y,z)));

    serviceTrajInit(false, target_pose, traj_duration);

    return true;
}


yarp::sig::Matrix Module::get_pose()
{
    yarp::sig::Matrix yarp_pose;

    yarp_pose.resize(4, 4);

    yarp::eigen::toEigen(yarp_pose) = getCurrPose().matrix();

    return yarp_pose;
}

bool Module::go_home()
{
    serviceTrajInit(false, home_pose_, min_traj_duration_);

    return true;
}

bool Module::is_motion_done()
{
    return getTrajIsEnded();
}

bool Module::ask_reachability_evaluation(const yarp::sig::Matrix &pose)
{
    if (getState() != State::Stop)
        return false;

    Eigen::Affine3d target_pose;
    target_pose.matrix() = yarp::eigen::toEigen(pose);

    serviceTrajInit(true, target_pose, getDuration());

    return true;
}

bool Module::is_pose_reachable(const double x, const double y, const double z, const double q_x, const double q_y, const double q_z, const double q_w)
{
    if (getState() != State::Stop)
        return false;

    Eigen::Affine3d target_pose;
    target_pose = Eigen::Translation3d(x, y, z);
    target_pose.rotate(Eigen::Quaterniond(q_w, q_x, q_y, q_z));

    serviceTrajInit(true, target_pose, getDuration());

    return true;
}

yarp::sig::Matrix Module::retrieve_reachable_pose()
{
    yarp::sig::Matrix reachability_result;

    if (!getTrajIsEnded())
    {
        reachability_result.resize(1, 1);
        return reachability_result;
    }

    reachability_result.resize(4, 4);
    yarp::eigen::toEigen(reachability_result) = desired_transform_.matrix();

    return reachability_result;
}

bool Module::stop()
{
    setState(State::Stop);
    setTrajIsEnded(true);

    return true;
}

bool Module::setJointsMode(const std::string &mode)
{
    if (getState() != State::Stop)
        return false;

    if (!cub_joint_control_.configureJointsMode(mode))
    {
        yError() << module_name_ + "::setJointsMode(). Error: Cannot set '" << mode << "' control mode. See the errors above.";
        return false;
    }

    return true;
}

void Module::serviceTrajInit(bool is_reach_eval, const Eigen::Affine3d& target_pose, double traj_duration)
{
    std::lock_guard<std::mutex> lg(mutex_);

    /* Initialize final pose and trajectory parameters that cannot be deferred. */
    traj_.is_ended = false;

    is_reach_eval_ = is_reach_eval;

    traj_.fin_pose = target_pose;

    if(traj_duration < 0.0)
    {
        yWarning() << module_name_ + "::serviceTrajInit(). traj_duration is " << traj_duration<< ", bit will be set to zero.";
        traj_.duration=0.0;
    }
    else
    {
        traj_.duration = traj_duration;
    }

    /* Going to complete the initialization inside TrajInit state. */
    state_ = State::TrajInit;
}
