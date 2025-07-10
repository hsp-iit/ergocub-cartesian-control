#include <module.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>

#include <utils/utils.h>
#include <utils/utils.hpp>

#include <optional>

#include <chrono>
#include <thread>

#include <unsupported/Eigen/MatrixFunctions>

using namespace std::literals::chrono_literals;

bool Module::configure(yarp::os::ResourceFinder &rf)
{
    /* Check for and retrieve mandatory parameters. */
    if (!(rf.check("rate") && rf.find("rate").isFloat64()))
    {
        yError() << module_name_ + "::configure(). Error: mandatory parameter 'rate' missing or invalid.";
        return false;
    }

    rate_ = rf.find("rate").asFloat64();
    sample_time_ = 1.0 / rate_;

    /* Check for and retrieve mandatory groups. */
    auto boolCheckAndRetrieve = [&](const yarp::os::ResourceFinder &rf_in, const std::string &param_name_in, bool &bool_out)
    {
        if (!(rf_in.check(param_name_in) && rf_in.find(param_name_in).isBool()))
        {
            yError() << module_name_ + "::configure(). Error: mandatory parameter '" + param_name_in + "' missing or invalid.";
            return false;
        }

        bool_out = rf_in.find(param_name_in).asBool();

        return true;
    };

    if (!boolCheckAndRetrieve(rf, "module_logging", module_logging_))
        return false;
    if (!boolCheckAndRetrieve(rf, "module_verbose", module_verbose_))
        return false;

    bool qp_verbose;
    if (!boolCheckAndRetrieve(rf, "qp_verbose", qp_verbose))
        return false;

    /* Check for and retrieve mandatory groups. */
    auto groupCheckAndRetrieve = [&](const yarp::os::ResourceFinder &rf_in, const std::string &group_name_in, yarp::os::Bottle &bottle_group_out)
    {
        if (!rf_in.check(group_name_in))
        {
            yError() << module_name_ + "::configure(). Error: mandatory group " + group_name_in + " missing.";
            return false;
        }

        bottle_group_out = rf_in.findGroup(group_name_in);

        return true;
    };

    yarp::os::Bottle ARM_bot;
    if (!groupCheckAndRetrieve(rf, "ARM", ARM_bot))
        return false;

    yarp::os::Bottle FK_PARAM_bot;
    if (!groupCheckAndRetrieve(rf, "FK_PARAM", FK_PARAM_bot))
        return false;

    yarp::os::Bottle IK_PARAM_bot;
    if (!groupCheckAndRetrieve(rf, "IK_PARAM", IK_PARAM_bot))
        return false;

    yarp::os::Bottle FSM_PARAM_bot;
    if (!groupCheckAndRetrieve(rf, "FSM_PARAM", FSM_PARAM_bot))
        return false;

    /* Instantiate CubJointControl controller. */
    if (!cub_joint_control_.configure(ARM_bot))
    {
        yError() << module_name_ + "::configure(). Error: cannot configure the cubJointControl instance. See the errors above.";
        return false;
    }

    if (!cub_joint_control_.configureJointsMode("Streaming"))
    {
        yError() << module_name_ + "::configure(). Error: Cannot set 'Streaming' control mode. See the errors above.";
        return false;
    }
    yInfo() << module_name_ + "::configure(). Set 'Streaming' control mode done.";

    /* Instantiate ProxQP-based differential inverse kinematics. */
    {
        /* Retrieve joint limits*/
        std::optional<std::unordered_map<std::string, Eigen::VectorXd>> joints_limit = cub_joint_control_.getJointLimits();

        if (!joints_limit.has_value())
        {
            yError() << module_name_ + "::configure(). Error: CubJointControl cannot retrieve joint limits.";
            return false;
        }

        const Eigen::VectorXd lower_limits = (*joints_limit)["lower"];
        const Eigen::VectorXd upper_limits = (*joints_limit)["upper"];

        auto extractFromBottle = [](const yarp::os::Bottle &b_in, Eigen::VectorXd &vec_out)
        {
            vec_out.resize(b_in.size());
            for (size_t i = 0; i < vec_out.size(); ++i)
            {
                vec_out[i] = b_in.get(i).asFloat64();
            }
        };

        /* Check for and retrieve ik values. */
        if (!(utils::checkParameters({{"limits_param"}}, "", IK_PARAM_bot, "", utils::ParameterType::Float64, false) &&
              (IK_PARAM_bot.find("limits_param").asFloat64() >= 0.0) && (IK_PARAM_bot.find("limits_param").asFloat64() <= 1.0)) ||
            !utils::checkParameters({{"max_joint_position_variation", "max_joint_position_track_error"}}, "", IK_PARAM_bot, "", utils::ParameterType::Float64, false) ||
            !utils::checkParameters({{"joint_vel_weight", "position_param", "orientation_param", "joint_pos_param"}}, "", IK_PARAM_bot, "", utils::ParameterType::Float64, true))
        {
            yError() << module_name_ + "::configure(). Error: mandatory parameter(s) for IK_PARAM group missing or invalid.";
            return false;
        }

        const double limits_param = IK_PARAM_bot.find("limits_param").asFloat64() / sample_time_;

        Eigen::VectorXd joint_vel_weight;
        extractFromBottle(*IK_PARAM_bot.find("joint_vel_weight").asList(), joint_vel_weight);

        Eigen::VectorXd position_param;
        extractFromBottle(*IK_PARAM_bot.find("position_param").asList(), position_param);

        Eigen::VectorXd orientation_param;
        extractFromBottle(*IK_PARAM_bot.find("orientation_param").asList(), orientation_param);

        Eigen::VectorXd joint_pos_param;
        extractFromBottle(*IK_PARAM_bot.find("joint_pos_param").asList(), joint_pos_param);

        max_joint_position_variation_ =  IK_PARAM_bot.find("max_joint_position_variation").asFloat64();

        max_joint_position_track_error_ =  IK_PARAM_bot.find("max_joint_position_track_error").asFloat64();

        if (!IK_PARAM_bot.check("joint_ref"))
        {
            joint_ref_values_ = cub_joint_control_.getJointRefValues();
            if(!joint_ref_values_.has_value())
            {
                yError() << module_name_ + "::configure(). Error: CubJointControl cannot retrieve last joint reference values. ";
                return false;
            }
        }
        else
        {
            if (!utils::checkParameters({{"joint_ref"}}, "", IK_PARAM_bot, "", utils::ParameterType::Float64, true))
            {
                yError() << module_name_ + "::configure(). Error: mandatory parameter 'joint_ref' for IK_PARAM_bot group missing or invalid.";
                return false;
            }

            yarp::os::Bottle *joint_ref_list = IK_PARAM_bot.find("joint_ref").asList();

            if ((joint_ref_list->size() != upper_limits.size()))
            {
                yError() << module_name_ + "::configure(). Error: joint_ref should be a list of size " << upper_limits.size() << ".";
                return false;
            }

            Eigen::VectorXd joint_ref(upper_limits.size());
            extractFromBottle(*joint_ref_list, joint_ref);
            joint_ref_values_ = joint_ref;
        }

        /* Initialize joint velocities. */
        Eigen::VectorXd init_val = Eigen::VectorXd::Zero(cub_joint_control_.getNumberJoints());
        ik_joint_vel_ = std::optional{init_val};

        /* Set ik solver. */
        ik_ = std::make_unique<DifferentialInverseKinematicsQP>(sample_time_, limits_param, joint_vel_weight, position_param, orientation_param, joint_pos_param, *joint_ref_values_, qp_verbose);
        ik_->set_joint_limits(lower_limits, upper_limits, limits_param * Eigen::VectorXd::Ones(upper_limits.size()));
    }

    /* Instantiate iDynTree-based forward kinematics. */
    {
        /* Check for and retrieve fk values. */
        if (!utils::checkParameters({{"root_frame_name", "ee_frame_name"}}, "", FK_PARAM_bot, "", utils::ParameterType::String, false))
        {
            yError() << module_name_ + "::configure(). Error: mandatory parameter(s) for FK_PARAM group missing or invalid.";
            return false;
        }
        const std::string root_frame_name = FK_PARAM_bot.find("root_frame_name").asString();
        const std::string ee_frame_name = FK_PARAM_bot.find("ee_frame_name").asString();

        /* No need to check here, arleady done above in CubJointControl. */
        std::vector<std::string> list_joints = utils::loadVectorString(ARM_bot, "joint_axes_list");

        /* Get robot urdf path. */
        const std::string robot_urdf_path = rf.findFileByName("model.urdf");

        if (robot_urdf_path.empty())
        {
            yError() << module_name_ + "::configure. Error: cannot load the robot urdf path. Please make sure that YARP_ROBOT_NAME environment variable is set. Make sure also that the YARP_ROBOT_NAME is correctly included in YARP_DATA_DIRS search path";
            return false;
        }

        /* Set fk solver. */
        fk_ = std::make_unique<ForwardKinematicsiDynTree>(robot_urdf_path, list_joints, root_frame_name, ee_frame_name);
    }

    /* Instantiate integrator_-> */
    integrator_ = std::make_unique<Integrator>(sample_time_);
    integrator_->set_initial_condition(*joint_ref_values_);

    /* Initialize desired transform = initial pose */
    fk_->set_joints_state(*joint_ref_values_);
    fk_->update();
    desired_transform_ = fk_->get_ee_transform();
    home_pose_ = desired_transform_;

    encoders_ = cub_joint_control_.getJointValues();
    if (!encoders_.has_value())
    {
        yError() << module_name_ + "::configure(). Error: CubJointControl cannot retrieve joint values.";
        return false;
    }

    fk_->set_joints_state(*encoders_);
    fk_->update();
    setCurrPose(fk_->get_ee_transform());


    /* Trajectory parameters initialization*/
    traj_.time = 0.0;

    if (!(rf.check("traj_duration") && rf.find("traj_duration").isFloat64()))
    {
        yError() << module_name_ + "::configure(). Error: mandatory parameter 'traj_duration' missing or invalid.";
        return false;
    }

    double duration = rf.find("traj_duration").asFloat64();

    if (duration < min_traj_duration_)
    {
        duration = min_traj_duration_;

        yWarning() << module_name_ + "::go_to_pose(). The requested duration is less than " + std::to_string(min_traj_duration_) + ". It will be enforced to that value.";
    }

    setDuration(duration);
    traj_.init_pose = desired_transform_;
    setTrajFinPose(desired_transform_);
    setTrajIsEnded(true);
    is_reach_eval_ = false;

    /* FSM */
    if (!utils::checkParameters({{"stop_speed"}}, "", FSM_PARAM_bot, "", utils::ParameterType::Float64, false))
    {
        yError() << module_name_ + "::configure(). Error: mandatory parameter(s) for FSM_PARAM group missing or invalid.";
        return false;
    }

    stop_speed_ = FSM_PARAM_bot.find("stop_speed").asFloat64();

    setState(State::Stop);

    /* Reachability*/
    if (!(rf.check("position_error_th") && rf.find("position_error_th").isFloat64() && (rf.find("position_error_th").asFloat64() > 0.0)))
    {
        yError() << module_name_ + "::configure(). Error: mandatory parameter 'position_error_th' missing or invalid.";
        return false;
    }
    if (!(rf.check("max_iteration") && rf.find("max_iteration").isInt32() && (rf.find("max_iteration").asInt32() > 0)))
    {
        yError() << module_name_ + "::configure(). Error: mandatory parameter 'max_iteration' missing or invalid.";
        return false;
    }

    pos_err_th_ = rf.find("position_error_th").asFloat64();
    max_iter_ = rf.find("max_iteration").asInt32();

    /* Configure RPC service. */
    if (!configureService(rf))
    {
        yError() << module_name_ + "::configure(). Error: cannot configure the RPC service.";
        return false;
    }

    if (module_logging_)
    {
        /* Open ports for logging */
        const std::string arm_name = ARM_bot.find("name").asString();
        if (!log_port_pose_.open("/gb-ergocub-cartesian-controller/log/" + arm_name + "/pose:o"))
        {
            yError() << module_name_ + "::configure(). Error: cannot open pose logging port.";
            return false;
        }

        if (!log_port_joint_.open("/gb-ergocub-cartesian-controller/log/" + arm_name + "/joints:o"))
        {
            yError() << module_name_ + "::configure(). Error: cannot open joints logging port.";
            return false;
        }

        /* Wait for the logger to connect to the logging ports*/
        yInfo() << "Module " + module_name_ + ":  wait 5s for the logger to connect to the logging ports.";
        std::this_thread::sleep_for(5s);
    }

    yInfo() << module_name_ + "::configure(): Configuration done.";

    return true;
}

bool Module::close()
{
    log_port_pose_.close();
    log_port_joint_.close();
    rpc_cmd_port_.close();

    return true;
}

double Module::getPeriod()
{
    return 1.0 / rate_;
}

bool Module::interruptModule()
{
    rpc_cmd_port_.interrupt();

    return true;
}

bool Module::updateModule()
{
    /* Read encoders. */
    encoders_ = cub_joint_control_.getJointValues();

    if (!encoders_.has_value())
    {
        yError() << module_name_ + "::updateModule(). Error: CubJointControl cannot retrieve joint values";
        error_prev_state_ = getState();
        setState(State::Error);
    }

    /* Update direct kinematics. */
    fk_->set_joints_state(*encoders_);
    fk_->update();
    setCurrPose(fk_->get_ee_transform());

    const State current_state = getState();

    if (current_state == State::TrajInit)
    {
        if (!cub_joint_control_.configureJointsMode("Streaming"))
        {
            yError() << module_name_ + "::updateModule(). Error: Cannot set 'Streaming' control mode. See the errors above.";
            error_prev_state_ = State::Running;
            setState(State::Error);
            return false;
        }

        const std::optional<Eigen::VectorXd> init_cub_joint_ref = cub_joint_control_.getJointRefValues();

        if(!init_cub_joint_ref.has_value())
        {
            yError() << module_name_ + "::updateModule(). Error: CubJointControl cannot retrieve last joint reference values. ";
            error_prev_state_ = State::Running;
            setState(State::Error);
            return false;
        }

        integrator_->set_initial_condition(*init_cub_joint_ref);

        fk_->set_joints_state(*init_cub_joint_ref);
        fk_->update();
        traj_.init_pose =  fk_->get_ee_transform();

        traj_.time = 0.0;


        if (is_reach_eval_)
            setState(State::ReachEval);
        else
            setState(State::Running);
    }
    else if (current_state == State::Running)
    {
        updateReferenceVelocities();

        if (!ik_joint_vel_.has_value())
        {
            yError() << module_name_ + "::updateModule(). Error: Inverse kinematics failed!";
            error_prev_state_ = State::Running;
            setState(State::Error);
            return false;
        }

        if (!checkInputAndUpdateIntegrator())
        {
            yError() << module_name_ + "::updateModule(). Error: At least one QP output requires a joint position reference variation greater than max_joint_position_variation!";
            error_prev_state_ = State::Running;
            setState(State::Error);
            return false;
        }

        /* Safety Check: future reference value doesn't cause an abrupt movement*/
        const Eigen::VectorXd future_ref = integrator_->get_state();
        for (int i = 0; i < future_ref.size(); ++i)
        {
            if (abs((*encoders_)(i)-future_ref(i)) > max_joint_position_track_error_ + max_joint_position_variation_)
            {
                yError() << module_name_ + "::updateModule(). Error: Low joint position tracking accuracy. At least one future joint position reference differs more than "+ std::to_string(max_joint_position_variation_ + max_joint_position_track_error_)+" w.r.t. the related joint position measured value!";
                error_prev_state_ = State::Running;
                setState(State::Error);
                return false;
            }
        }

        /* Send the integral to the robot. */
        if (!cub_joint_control_.configureJointsMode("Streaming"))
        {
            yError() << module_name_ + "::updateModule(). Error: Cannot set 'Streaming' control mode. See the errors above.";
            error_prev_state_ = State::Running;
            setState(State::Error);
            return false;
        }

        if (!cub_joint_control_.moveToStreaming(future_ref))
        {
            yError() << module_name_ + "::updateModule(). Error: Cannot set desired joint position reference. See the errors above.";
            error_prev_state_ = State::Running;
            setState(State::Error);
            return false;
        }

        if (traj_.time < traj_.duration)
            traj_.time += sample_time_;
        else
        {
            bool is_ended = true;
            auto ik_joint_vel_abs = (*ik_joint_vel_).array().abs();

            for (int i = 0; i < ik_joint_vel_abs.size(); ++i)
            {
                if (ik_joint_vel_abs(i) > stop_speed_)
                {
                    is_ended = false;
                    break;
                }
            }

            if (is_ended)
            {
                setTrajIsEnded(is_ended);
                setState(State::Stop);
            }
        }
    }
    else if (current_state == State::ReachEval)
    {
        int iteration = 1;
        double position_error = (traj_.fin_pose.translation() - traj_.init_pose.translation()).norm();

        while ((traj_.time <= traj_.duration) && (iteration <= max_iter_) && (position_error > pos_err_th_))
        {
            updateReferenceVelocities();

            if (!ik_joint_vel_.has_value())
            {
                yError() << module_name_ + "::reachable_pose(). Error: Inverse kinematics failed!";
                error_prev_state_ = State::ReachEval;
                setState(State::Error);
                return false;
            }

            if (!checkInputAndUpdateIntegrator())
            {
                yError() << module_name_ + "::reachable_pose(). Error: At least one QP output requires a joint position reference variation greater than max_joint_position_variation!";
                error_prev_state_ = State::ReachEval;
                setState(State::Error);
                return false;
            }

            traj_.time += sample_time_;
            iteration++;
            position_error = (traj_.fin_pose.translation() - desired_transform_.translation()).norm();
        }

        if (module_verbose_)
        {
            yInfo() << "position_error | pos_err_th_\t" << position_error << "|" << pos_err_th_;
            yInfo() << "iteration | max_iter_\t" << iteration << "|" << max_iter_;
            yInfo() << "time | duration\t" << traj_.time << "|" << traj_.duration;
        }

        setTrajIsEnded(true);
        setState(State::Stop);
    }
    else if (current_state == State::Error)
    {
        // TO DO: manage the error with error_prev_state_
    }
    else if (current_state == State::Stop)
    {
    }
    else
    {
        yWarning() << "**********State::Unknown**********";
    }

    if (module_logging_)
        log();

    return true;
}

void Module::setCurrPose(const Eigen::Transform<double, 3, Eigen::Affine> &curr_pose)
{
    std::lock_guard<std::mutex> lg(mutex_);

    current_transform_ = curr_pose;

}

Eigen::Transform<double, 3, Eigen::Affine> Module::getCurrPose()
{
    std::lock_guard<std::mutex> lg(mutex_);

    return current_transform_;
}

void Module::setTrajIsEnded(bool is_ended)
{
    std::lock_guard<std::mutex> lg(mutex_);

    traj_.is_ended = is_ended;
}

bool Module::getTrajIsEnded()
{
    std::lock_guard<std::mutex> lg(mutex_);

    return traj_.is_ended;
}

void Module::setDuration(double duration)
{
    std::lock_guard<std::mutex> lg(mutex_);

    traj_.duration = duration;
}

double Module::getDuration()
{
    std::lock_guard<std::mutex> lg(mutex_);

    return traj_.duration;
}

void Module::setTrajFinPose(const Eigen::Transform<double, 3, Eigen::Affine> &fin_pose)
{
    std::lock_guard<std::mutex> lg(mutex_);

    traj_.fin_pose = fin_pose;
}

void Module::setTrajFinPose(const Eigen::Matrix4d &fin_pose)
{
    std::lock_guard<std::mutex> lg(mutex_);

    traj_.fin_pose.matrix() = fin_pose;
}

Eigen::Transform<double, 3, Eigen::Affine> Module::getTrajFinPose()
{
    std::lock_guard<std::mutex> lg(mutex_);

    return traj_.fin_pose;
}

void Module::setState(const State &des_state)
{
    const std::lock_guard<std::mutex> lock(mutex_);

    state_ = des_state;
}

Module::State Module::getState()
{
    const std::lock_guard<std::mutex> lock(mutex_);

    return state_;
}

void Module::updateReferenceVelocities()
{
    /* Get the previous state of the integrator */
    const Eigen::VectorXd q_prev = integrator_->get_state();

    /* Get the previous input of the integrator */
    const Eigen::VectorXd dq_prev = *ik_joint_vel_;

    /* Set joints and update fk_-> */
    fk_->set_joints_state(q_prev);
    fk_->update();

    /* Get forward kinematics. */
    auto transform = fk_->get_ee_transform();

    /* Get jacobian. */
    auto jacobian = fk_->get_jacobian();

    /* Set robot state in the ik_-> */
    ik_->set_robot_state(q_prev, dq_prev, transform, jacobian);

    /* Evaluate trajectory */
    Eigen::Affine3d fin_pose = getTrajFinPose();
    double traj_duration = getDuration();
    desired_transform_ = Eigen::Translation3d(minJerkLinTraj(traj_.init_pose.translation(), fin_pose.translation(), traj_duration, traj_.time));
    desired_transform_.rotate(minJerkAngTraj(traj_.init_pose.rotation(), fin_pose.rotation(), traj_duration, traj_.time));

    /* Set desired transform in the ik_-> */
    ik_->set_desired_ee_transform(desired_transform_);

    /* Solve the ik_-> and retrieve solution*/
    ik_joint_vel_ = ik_->eval_reference_velocities();
}

bool Module::checkInputAndUpdateIntegrator()
{
    /* Safety Check: QP output doesn't cause an abrupt movement. */
    auto ik_joint_pos_var = sample_time_ * (*ik_joint_vel_).array().abs();
    for (int i = 0; i < ik_joint_pos_var.size(); ++i)
    {
        if (ik_joint_pos_var(i) > max_joint_position_variation_)
        {
            return false;
        }
    }

    /* Evaluate the integral of the reference velocities. */
    integrator_->integrate(*ik_joint_vel_);

    return true;
}

void Module::log()
{
    if (!encoders_.has_value())
    {
        yError() << module_name_ + "::log(). Error: CubJointControl cannot retrieve joint values.";
        return;
    }

    Eigen::Affine3d current_transform = getCurrPose();



    /* QP-pose */
    fk_->set_joints_state(integrator_->get_state());
    fk_->update();
    Eigen::Affine3d qp_transform = fk_->get_ee_transform();

    /* Joints error. */
    Eigen::VectorXd e_joints = integrator_->get_state() - *encoders_;

    /* Positional error. */
    Eigen::VectorXd e_pos = desired_transform_.translation() - current_transform.translation();

    Eigen::VectorXd e_qp_pos = desired_transform_.translation() - qp_transform.translation();

    /* Rotational error. */
    Eigen::AngleAxisd e_rot_aa(desired_transform_.rotation() * current_transform.rotation().transpose());
    Eigen::Vector3d e_rot = e_rot_aa.axis() * e_rot_aa.angle();

    Eigen::AngleAxisd e_qp_rot_aa(desired_transform_.rotation() * qp_transform.rotation().transpose());
    Eigen::Vector3d e_qp_rot = e_qp_rot_aa.axis() * e_qp_rot_aa.angle();

    /* Define logging msgs */
    yarp::sig::Vector &pose_msg = log_port_pose_.prepare();
    yarp::sig::Vector &joint_msg = log_port_joint_.prepare();

    pose_msg.resize(desired_transform_.translation().size() + current_transform.translation().size() + e_pos.size() + 2);
    joint_msg.resize(integrator_->get_state().size() + e_joints.size() + (*ik_joint_vel_).size() + 1);

    /* Lambda function to fill a message*/
    auto fillMsg = [](yarp::sig::Vector &msg_in, const Eigen::VectorXd &vec_in, int offset = 0)
    {
        for (int i = 0; i < vec_in.size(); ++i)
        {
            msg_in[i + offset] = vec_in[i];
        }
    };

    /* Fill pose msg*/
    fillMsg(pose_msg, desired_transform_.translation());
    int offset = desired_transform_.translation().size();
    fillMsg(pose_msg, current_transform.translation(), offset);
    offset += current_transform.translation().size();
    fillMsg(pose_msg, e_pos, offset);
    offset += e_pos.size();
    pose_msg[offset] = e_pos.norm();
    ++offset;
    pose_msg[offset] = e_rot_aa.angle() * (180.0 / M_PI);

    /* Fill joints msg*/
    fillMsg(joint_msg, integrator_->get_state());
    offset = integrator_->get_state().size();
    fillMsg(joint_msg, e_joints * (180.0 / M_PI), offset);
    offset += e_joints.size();
    fillMsg(joint_msg, (*ik_joint_vel_) * (180.0 / M_PI), offset);
    offset += (*ik_joint_vel_).size();
    joint_msg[offset] = (int)getState();

    /* Send msgs*/
    log_port_pose_.write();
    log_port_joint_.write();

    if (module_verbose_)
    {
        /* Print some info */
        auto state = getState();
        if (state == State::Stop)
            {yInfo() << "state\t|Stop|";}
        else if (state == State::Running)
            yInfo() << "state\t|Running|";
        else if (state == State::TrajInit)
            yInfo() << "state\t|TrajInit|";
        else if (state == State::ReachEval)
            yInfo() << "state\t|ReachEval|";
        else if (state == State::Error)
            yInfo() << "state\t|Error|";
        else
            yInfo() << "state\t|Unknown|";

        /* From Eigen to string. */
        auto eigenToString = [] (const Eigen::MatrixXd& eigen_in)
        {
            std::ostringstream ss;

            ss <<"[";
            int i;
            for (i = 0; i < eigen_in.size() - 1; i++)
            {
                ss<<eigen_in(i)<<", ";
            }
            ss<<eigen_in(i)<<"]";

            return ss.str();
        };

        //desired values
        Eigen::AngleAxisd des_rot ( desired_transform_.rotation());
        yInfo() << "---------- Desired min-jerk trajectory --------------------------";
        yInfo() << "pos des" << eigenToString(desired_transform_.translation());
        yInfo() << "ori des" << eigenToString(des_rot.axis() * des_rot.angle());

        //QP generated values
        fk_->set_joints_state(integrator_->get_state());
        fk_->update();
        auto qp_trf = fk_->get_ee_transform();
        auto qp_pos = qp_trf.translation();
        auto qp_rot = qp_trf.rotation();
        Eigen::AngleAxisd qp_rot_aa (qp_rot);

        Eigen::VectorXd qp_e_pos = desired_transform_.translation() - qp_pos;
        Eigen::AngleAxisd qp_e_rot_aa(desired_transform_.rotation() * qp_rot.transpose());
        Eigen::Vector3d qp_e_rot = qp_e_rot_aa.axis() * qp_e_rot_aa.angle();

        auto jacobian = fk_->get_jacobian();
        static double max_manip = 0.0;

        double manip = sqrt((jacobian * jacobian.transpose()).determinant());
        if (max_manip < manip) max_manip = manip;
        double weight_manip_function =  pow(1 - (manip / max_manip), 2);

        yInfo() << "---------- QP generated values, errors w.r.t. min-jerk ----------";
        yInfo() << "pos gen" << eigenToString(qp_pos);
        yInfo() << "ori gen" << eigenToString(qp_rot_aa.axis() * qp_rot_aa.angle());
        yInfo() << "pos err (m) |norm| [components]" << qp_e_pos.norm() << "\t" << eigenToString(qp_e_pos);
        yInfo() << "ang err (deg) |norm| [components]" << qp_e_rot_aa.angle() * (180 / M_PI) << "\t" << eigenToString(qp_e_rot);
        yInfo() << "QP des vel" << eigenToString(*ik_joint_vel_);
        yInfo() << "joints gen" << eigenToString(integrator_->get_state());
        yInfo() << "joints ref" << eigenToString(*joint_ref_values_);
        yInfo() << "joints err g-r [rad]" << eigenToString(integrator_->get_state() - *joint_ref_values_);
        yInfo() << "joints err g-r [deg]" << eigenToString((integrator_->get_state() - *joint_ref_values_)* (180 / M_PI));
        yInfo() << "manip "<<manip<<" max_manip "<<max_manip<<" manip/max_manip "<<manip/max_manip<<" weight_manip_function "<<weight_manip_function;

        //measured values
        Eigen::Affine3d cur_trf = getCurrPose();
        Eigen::VectorXd cur_pos = cur_trf.translation();
        auto cur_rot = cur_trf.rotation();
        Eigen::AngleAxisd cur_rot_aa (cur_rot);

        Eigen::VectorXd cur_e_pos = qp_pos - cur_pos;
        Eigen::AngleAxisd cur_e_rot_aa(qp_rot * cur_rot.transpose());
        Eigen::Vector3d cur_e_rot = cur_e_rot_aa.axis() * cur_e_rot_aa.angle();


        yInfo() << "---------- Current values, errors w.r.t. QP generated -----------";
        yInfo() << "pos cur" << eigenToString(cur_pos);
        yInfo() << "ori cur" << eigenToString(cur_rot_aa.axis() * cur_rot_aa.angle());
        yInfo() << "pos err (m) |norm| [components]" << cur_e_pos.norm() << "\t" << eigenToString(cur_e_pos);
        yInfo() << "ang err (deg) |norm| [components]" << cur_e_rot_aa.angle() * (180 / M_PI) << "\t" << eigenToString(cur_e_rot);
        yInfo() << "joints cur" << eigenToString(*encoders_);
        yInfo() << "joints err" << eigenToString(integrator_->get_state() - *encoders_);
        yInfo() << "################################################################";
    }
}

Eigen::Vector3d minJerkLinTraj(const Eigen::Vector3d &vec_i, const Eigen::Vector3d &vec_f, double duration, double time)
{
    return Eigen::Vector3d(evalMinJerk(vec_i.x(), vec_f.x(), duration, time),
                           evalMinJerk(vec_i.y(), vec_f.y(), duration, time),
                           evalMinJerk(vec_i.z(), vec_f.z(), duration, time));
}

Eigen::Matrix3d minJerkAngTraj(const Eigen::Matrix3d &mat_i, const Eigen::Matrix3d &mat_f, double duration, double time)
{
    /*  Reference: https://ami-iit.github.io/bipedal-locomotion-framework/so3-minjerk.html

        Notice that in the following the trajectory is generated w.r.t. the robot frame
        while the reference's formula is expressed w.r.t. the end-effector frame.
        */
    Eigen::AngleAxisd aa(mat_f * mat_i.transpose());

    Eigen::Matrix3d log = skew(aa.axis() * aa.angle());

    double s_t = evalMinJerk(0, 1, duration, time);

    Eigen::Matrix3d R_t = (s_t * log).exp() * mat_i;

    return R_t;
}

double evalMinJerk(double q_i, double q_f, double duration, double time)
{
    double q_time;

    if (time > duration)
    {
        q_time = q_f;
    }
    else
    {
        double dq = q_f - q_i;
        q_time = q_i + (10 * dq * std::pow(time, 3) / std::pow(duration, 3)) - (15 * dq * std::pow(time, 4) / std::pow(duration, 4)) + (6 * dq * std::pow(time, 5) / std::pow(duration, 5));
    }

    return q_time;
}

Eigen::Matrix3d skew(const Eigen::Vector3d &vec)
{
    Eigen::Matrix3d S;

    S(0, 0) = 0;
    S(0, 1) = -vec.z();
    S(0, 2) = vec.y();
    S(1, 0) = vec.z();
    S(1, 1) = 0;
    S(1, 2) = -vec.x();
    S(2, 0) = -vec.y();
    S(2, 1) = vec.x();
    S(2, 2) = 0;

    return S;
}
