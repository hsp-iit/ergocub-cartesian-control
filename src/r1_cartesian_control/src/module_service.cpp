#include <module.h>

#include <yarp/os/LogStream.h>
#include <yarp/eigen/Eigen.h>

bool Module::configureService(const yarp::os::ResourceFinder &rf)
{
    if (!rf.check("rpc_local_port_name") || !rf.find("rpc_local_port_name").isString())
    {
        yError() << module_name_ + "::configureService(). Error: missing or invalid rpc_port_name parameter.";
        return false;
    }

    std::string rpc_port_name = rf.find("rpc_local_port_name").asString();

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
    yDebug() << "Go to pose: " << x << " " << y << " " << z << " " << q_x << " " << q_y << " " << q_z << " " << q_w;
    Eigen::Affine3d target_pose;
    target_pose = Eigen::Translation3d(x, y, z);
    target_pose.rotate(Eigen::Quaterniond(q_w, q_x, q_y, q_z));

    serviceTrajInit(false, target_pose, traj_duration);

    return true;
}

bool Module::go_to_position(double x, double y, double z, double traj_duration)
{
    if (getState() != State::Stop)
        return false;

    Eigen::Affine3d target_pose = getCurrPose();
    target_pose.translation() = Eigen::Vector3d(x, y, z);

    serviceTrajInit(false, target_pose, traj_duration);

    return true;
}

bool Module::go_home()
{
    if (getState() != State::Stop)
        return false;

    serviceTrajInit(false, home_pose_, getDuration());

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


bool Module::stop()
{
    setState(State::Stop);
    setTrajIsEnded(true);

    return true;
}

void Module::serviceTrajInit(bool is_reach_eval, const Eigen::Affine3d& target_pose, double traj_duration)
{
    std::lock_guard<std::mutex> lg(mutex_);

    /* Initialize final pose and trajectory parameters that cannot be deferred. */
    traj_.is_ended = false;

    is_reach_eval_ = is_reach_eval;

    traj_.fin_pose = target_pose;

    if (traj_duration < min_traj_duration_)
    {
        traj_duration = min_traj_duration_;

        yWarning() << module_name_ + "::serviceTrajInit(). The requested duration is less than " + std::to_string(min_traj_duration_) + ". It will be enforced to that value.";
    }

    traj_.duration = traj_duration;

    /* Going to complete the initialization inside TrajInit state. */
    state_ = State::TrajInit;
}
