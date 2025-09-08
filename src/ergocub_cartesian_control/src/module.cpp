// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include <module.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

#include <utils/utils.h>
#include <utils/utils.hpp>

#include <optional>

#include <chrono>
#include <thread>

#include <unsupported/Eigen/MatrixFunctions>

using namespace std::literals::chrono_literals;

bool Module::configure(yarp::os::ResourceFinder &rf)
{
    
    /* Check for and retrieve mandatory groups. */
    auto groupCheckAndRetrieve = [&](const yarp::os::Searchable &src,
                                 const std::string           &group_name_in,
                                 yarp::os::Bottle            &bottle_group_out) -> bool
    {
    if (!src.check(group_name_in))
    {
        yError() << module_name_ + "::configure(). Error: mandatory group "
                 << group_name_in << " missing.";
        return false;
    }

    bottle_group_out = src.findGroup(group_name_in);  // copia profonda
    return true;
    };

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

    yarp::os::Bottle COMMON_bot;
    if (!groupCheckAndRetrieve(rf, "COMMON", COMMON_bot))
        return false;

    if  (   !(utils::checkParameters({{"rate", "traj_duration", "position_error_th"}}, "", COMMON_bot, "", utils::ParameterType::Float64, false))
        ||  !(utils::checkParameters({{"module_logging", "module_verbose", "qp_verbose"}}, "", COMMON_bot, "", utils::ParameterType::Bool, false))
        /*||  !(utils::checkParameters({{"rpc_local_port_name" }}, "", COMMON_bot, "", utils::ParameterType::String, false))*/)
    {
        yError() << "[" + module_name_ + "::configure] Error: mandatory parameter(s) for COMMON group missing or invalid.";
        return false;
    }

    // Assign values from COMMON_bot to the module variables.
    sample_time_ = 1.0 / COMMON_bot.find("rate").asFloat64();
    module_logging_ = COMMON_bot.find("module_logging").asBool();
    module_verbose_ = COMMON_bot.find("module_verbose").asBool();
    const bool qp_verbose = COMMON_bot.find("qp_verbose").asBool();
    const std::string rpc_local_port_name = COMMON_bot.find("rpc_local_port_name").asString();
    double duration = COMMON_bot.find("traj_duration").asFloat64();
    if (duration < min_traj_duration_)
    {
        duration = min_traj_duration_;

        yWarning() << module_name_ + "::go_to_pose(). The requested duration is less than " + std::to_string(min_traj_duration_) + ". It will be enforced to that value.";
    }
    setDuration(duration);
    pos_err_th_ = COMMON_bot.find("position_error_th").asFloat64();
    if (pos_err_th_ <= 0.0)
    {
        yError() << module_name_ + "::configure(). Error: mandatory parameter 'position_error_th' should be greater than 0.0.";
        return false;
    }
    max_iter_ = COMMON_bot.find("max_iteration").asInt32();
    if (max_iter_ <= 0)
    {
        yError() << module_name_ + "::configure(). Error: mandatory parameter 'max_iteration' should be greater than 0.";
        return false;
    }
    
    //-------------------------------------------
    //  Log configuration parameters
    //-------------------------------------------
    yInfo() << module_name_ << "::configure() ― parameters loaded:"
            << "\n  sampling_time         : " << sample_time_
            << "\n  module_logging        : " << std::boolalpha << module_logging_
            << "\n  module_verbose        : " << std::boolalpha << module_verbose_
            << "\n  qp_verbose            : " << std::boolalpha << qp_verbose
            << "\n  rpc_local_port_name   : " << rpc_local_port_name
            << "\n  traj_duration [s]     : " << duration
            << "\n  position_error_th [m] : " << max_iter_
            << "\n  max_iteration         : " << pos_err_th_;




    yarp::os::Bottle FK_PARAM_bot;
    if (!groupCheckAndRetrieve(rf, "FK_PARAM", FK_PARAM_bot))
        return false;

    yarp::os::Bottle FSM_PARAM_bot;
    if (!groupCheckAndRetrieve(rf, "FSM_PARAM", FSM_PARAM_bot))
        return false;

    yarp::os::Bottle ARM_bot;
    if (!groupCheckAndRetrieve(rf, "ARM", ARM_bot))
        return false;

    

    yarp::os::Bottle IK_PARAM_bot;
    if (!groupCheckAndRetrieve(rf, "IK_PARAM", IK_PARAM_bot))
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
            !utils::checkParameters({{"joint_acc_weight", "position_param", "orientation_param", "joint_pos_param"}}, "", IK_PARAM_bot, "", utils::ParameterType::Float64, true) ||
            !utils::checkParameters({{"torso_joints_to_stiffen"}}, "", IK_PARAM_bot, "", utils::ParameterType::Int32, false))
        {
            yError() << module_name_ + "::configure(). Error: mandatory parameter(s) for IK_PARAM group missing or invalid.";
            return false;
        }

        const double limits_param = IK_PARAM_bot.find("limits_param").asFloat64();
        double improve_manip_dyn = IK_PARAM_bot.find("improve_manip_dyn").asFloat64();
        double improve_manip_th = IK_PARAM_bot.find("improve_manip_th").asFloat64();

        Eigen::VectorXd joint_acc_weight;
        extractFromBottle(*IK_PARAM_bot.find("joint_acc_weight").asList(), joint_acc_weight);

        Eigen::VectorXd position_param;
        extractFromBottle(*IK_PARAM_bot.find("position_param").asList(), position_param);

        Eigen::VectorXd orientation_param;
        extractFromBottle(*IK_PARAM_bot.find("orientation_param").asList(), orientation_param);

        Eigen::VectorXd joint_pos_param;
        extractFromBottle(*IK_PARAM_bot.find("joint_pos_param").asList(), joint_pos_param);

        int torso_joints_to_stiffen = IK_PARAM_bot.find("torso_joints_to_stiffen").asInt32();

        max_joint_position_variation_ =  IK_PARAM_bot.find("max_joint_position_variation").asFloat64();

        max_joint_position_track_error_ =  IK_PARAM_bot.find("max_joint_position_track_error").asFloat64();

        if (!IK_PARAM_bot.check("joint_home"))
        {
            joint_home_values_ = cub_joint_control_.getJointRefValues();

            if(!joint_home_values_.has_value())
            {
                yError() << module_name_ + "::configure(). Error: CubJointControl cannot retrieve last joint reference values. ";
                return false;
            }
        }
        else
        {
            if (!utils::checkParameters({{"joint_home"}}, "", IK_PARAM_bot, "", utils::ParameterType::Float64, true))
            {
                yError() << module_name_ + "::configure(). Error: mandatory parameter 'joint_home' for IK_PARAM_bot group missing or invalid.";
                return false;
            }

            yarp::os::Bottle *joint_ref_list = IK_PARAM_bot.find("joint_home").asList();

            if ((joint_ref_list->size() != upper_limits.size()))
            {
                yError() << module_name_ + "::configure(). Error: joint_home should be a list of size " << upper_limits.size() << ".";
                return false;
            }

            Eigen::VectorXd joint_home(upper_limits.size());
            extractFromBottle(*joint_ref_list, joint_home);
            joint_home_values_ = joint_home;
        }

        /* Initialize joint velocities and accelerations. */
        ik_joint_vel_ = Eigen::VectorXd::Zero(cub_joint_control_.getNumberJoints());
        ik_joint_acc_ = Eigen::VectorXd::Zero(cub_joint_control_.getNumberJoints());

        /* Set ik solver. */
        ik_ = std::make_unique<DifferentialInverseKinematicsQP>(sample_time_, limits_param, joint_acc_weight, position_param, orientation_param, joint_pos_param, *joint_home_values_, improve_manip_dyn, improve_manip_th, torso_joints_to_stiffen, qp_verbose);
        ik_->set_joint_limits(lower_limits, upper_limits, limits_param * Eigen::VectorXd::Ones(upper_limits.size()));
        // ik_->setManipImproveGains(improve_manip_dyn, improve_manip_th);
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
            yError() << module_name_ + "::configure. Error: cannot load the robot urdf path. Please chack that the YARP_ROBOT_NAME environment variable is set.";
            return false;
        }

        /* Set fk solver. */
        fk_ = std::make_unique<ForwardKinematicsiDynTree>(robot_urdf_path, list_joints, root_frame_name, ee_frame_name);
    }

    /* Instantiate vel2pos_integrator_-> */
    vel2pos_integrator_ = std::make_unique<Integrator>(sample_time_);
    vel2pos_integrator_->set_initial_condition(*joint_home_values_);

    acc2vel_integrator_ = std::make_unique<Integrator>(sample_time_);
    acc2vel_integrator_->set_initial_condition(Eigen::VectorXd::Zero(joint_home_values_.value().size()));

    /* Initialize desired transform = initial pose */
    Eigen::VectorXd joint_zero(Eigen::VectorXd::Zero((*joint_home_values_).size()));
    fk_->set_joints_state(*joint_home_values_, joint_zero, joint_zero);
    fk_->update();
    desired_transform_ = fk_->get_ee_transform();
    home_pose_ = desired_transform_;

    encoders_pos_ = cub_joint_control_.getJointValues();
    if (!encoders_pos_.has_value())
    {
        yError() << module_name_ + "::configure(). Error: CubJointControl cannot retrieve joint values.";
        return false;
    }

    encoders_vel_ = cub_joint_control_.getJointSpeeds();
    if (!encoders_vel_.has_value())
    {
        yError() << module_name_ + "::configure(). Error: CubJointControl cannot retrieve joint speeds.";
        return false;
    }

    encoders_acc_ = cub_joint_control_.getJointAccelerations();
    if (!encoders_acc_.has_value())
    {
        yError() << module_name_ + "::configure(). Error: CubJointControl cannot retrieve joint accelerations.";
        return false;
    }


    fk_->set_joints_state(*encoders_pos_, *encoders_vel_, *encoders_acc_);
    fk_->update();
    setCurrPose(fk_->get_ee_transform());

    traj_.init_pose = desired_transform_;
    setTrajFinPose(desired_transform_);
    setTrajIsEnded(true);
    is_reach_eval_ = false;

    traj_.lin_gen = std::make_unique<PositionTrajectory>(sample_time_);
    traj_.ang_gen = std::make_unique<OrientationTrajectory>(sample_time_);
    desired_lin_vel_ = Eigen::Vector3d::Zero();
    desired_ang_vel_ = Eigen::Vector3d::Zero();
    
    /* FSM */
    if (!utils::checkParameters({{"stop_vel"}}, "", FSM_PARAM_bot, "", utils::ParameterType::Float64, false))
    {
        yError() << module_name_ + "::configure(). Error: mandatory parameter(s) for FSM_PARAM group missing or invalid.";
        return false;
    }

    stop_vel_ = FSM_PARAM_bot.find("stop_vel").asFloat64();

    setState(State::Stop);

    /* Configure RPC service. */
    if (!configureService(rf, rpc_local_port_name))
    {
        yError() << module_name_ + "::configure(). Error: cannot configure the RPC service.";
        return false;
    }

    if (module_logging_)
    {
        /* Open ports for logging */
        const std::string arm_name = ARM_bot.find("name").asString();

        auto loggerOption = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>(rf);
        loggerOption->setParameter("remote", "/" + module_name_ + "/" + arm_name + "/logger");

        if (!m_vectorsCollectionServer.initialize(loggerOption))
        {
            yError() << "Module " + module_name_ + ". Error: cannot initialize m_vectorsCollectionServer.";
            return false;
        }

        m_vectorsCollectionServer.populateMetadata("cartesian_state", {"state"});

        m_vectorsCollectionServer.populateMetadata("trajGen::pos::ini", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("trajGen::pos::fin", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("trajGen::pos::des", {"x", "y", "z"});

        m_vectorsCollectionServer.populateMetadata("trajGen::ori::ini", {"Y", "P", "R"});
        m_vectorsCollectionServer.populateMetadata("trajGen::ori::fin", {"Y", "P", "R"});
        m_vectorsCollectionServer.populateMetadata("trajGen::ori::des", {"Y", "P", "R"});

        m_vectorsCollectionServer.populateMetadata("qpIK::joints::pos", {"torso_roll", "torso_pitch", "torso_yaw", "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_yaw", "wrist_roll", "wrist_pitch"});
        m_vectorsCollectionServer.populateMetadata("qpIK::joints::vel", {"torso_roll", "torso_pitch", "torso_yaw", "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_yaw", "wrist_roll", "wrist_pitch"});
        m_vectorsCollectionServer.populateMetadata("qpIK::joints::acc", {"torso_roll", "torso_pitch", "torso_yaw", "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_yaw", "wrist_roll", "wrist_pitch"});
        m_vectorsCollectionServer.populateMetadata("qpIK::cartes::pos", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("qpIK::cartes::ori", {"Y", "P", "R"});
        m_vectorsCollectionServer.populateMetadata("qpIK::manips", {"manip", "max_manip", "manip_weight"});

        m_vectorsCollectionServer.populateMetadata("measured::joints::pos", {"torso_roll", "torso_pitch", "torso_yaw", "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_yaw", "wrist_roll", "wrist_pitch"});
        m_vectorsCollectionServer.populateMetadata("measured::joints::vel", {"torso_roll", "torso_pitch", "torso_yaw", "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_yaw", "wrist_roll", "wrist_pitch"});
        m_vectorsCollectionServer.populateMetadata("measured::joints::acc", {"torso_roll", "torso_pitch", "torso_yaw", "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_yaw", "wrist_roll", "wrist_pitch"});
        m_vectorsCollectionServer.populateMetadata("measured::cartes::pos", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("measured::cartes::ori", {"Y", "P", "R"});
        m_vectorsCollectionServer.populateMetadata("measured::manips", {"manip", "max_manip", "manip_weight"});

        m_vectorsCollectionServer.populateMetadata("reference::joints::pos", {"torso_roll", "torso_pitch", "torso_yaw", "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_yaw", "wrist_roll", "wrist_pitch"});


        m_vectorsCollectionServer.finalizeMetadata();

    }

    yInfo() << module_name_ + "::configure(): Configuration done.";

    return true;
}

bool Module::close()
{
    rpc_cmd_port_.close();

    return true;
}

double Module::getPeriod()
{
    return sample_time_;
}

bool Module::interruptModule()
{
    rpc_cmd_port_.interrupt();

    return true;
}

bool Module::updateModule()
{
    /* Read encoders. */
    encoders_pos_ = cub_joint_control_.getJointValues();

    if (!encoders_pos_.has_value())
    {
        yError() << module_name_ + "::updateModule(). Error: CubJointControl cannot retrieve joint values";
        error_prev_state_ = getState();
        setState(State::Error);
    }

    encoders_vel_ = cub_joint_control_.getJointSpeeds();

    if (!encoders_vel_.has_value())
    {
        yError() << module_name_ + "::updateModule(). Error: CubJointControl cannot retrieve joint speeds.";
        return false;
    }

    encoders_acc_ = cub_joint_control_.getJointAccelerations();
    if (!encoders_acc_.has_value())
    {
        yError() << module_name_ + "::updateModule(). Error: CubJointControl cannot retrieve joint accelerations.";
        return false;
    }

    /* Update direct kinematics. */
    fk_->set_joints_state(*encoders_pos_, *encoders_vel_, *encoders_acc_);
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

        auto init_joint_ref = cub_joint_control_.getJointRefValues();
        if(!init_joint_ref.has_value())
        {
            yError() << module_name_ + "::updateModule(). Error: Cannot retrieve joint reference values.";
            error_prev_state_ = State::Running;
            setState(State::Error);
            return false;
        }

        vel2pos_integrator_->set_initial_condition(*init_joint_ref);
        acc2vel_integrator_->set_initial_condition(ik_joint_vel_);
        //ik_joint_acc_ = *ik_joint_acc_;

        fk_->set_joints_state(vel2pos_integrator_->get_state(), acc2vel_integrator_->get_state(), *ik_joint_acc_);
        fk_->update();
        traj_.init_pose = fk_->get_ee_transform();

        auto duration = getDuration();

        if(duration != 0.0)
        {
            Eigen::Vector3d init_lin_vel = fk_->get_ee_lin_vel();
            Eigen::Vector3d init_ang_vel = fk_->get_ee_ang_vel();

            Eigen::Vector3d ini_lin_acc = fk_->get_ee_lin_acc();
            Eigen::Vector3d ini_ang_acc = fk_->get_ee_ang_acc();

            traj_.lin_gen->init(duration, getTrajFinPose().translation(), traj_.init_pose.translation(), init_lin_vel, ini_lin_acc);
            traj_.ang_gen->init(duration, getTrajFinPose().rotation(), traj_.init_pose.rotation(), init_ang_vel, ini_ang_acc);
        }

        if (is_reach_eval_)
            setState(State::ReachEval);
        else
            setState(State::Running);
    }
    else if (current_state == State::Running)
    {
        if(!updateReferenceVelocities())
        {
            yError() << module_name_ + "::updateModule(). See error(s) above.";
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
        const Eigen::VectorXd future_ref = vel2pos_integrator_->get_state();
        for (int i = 0; i < future_ref.size(); ++i)
        {
            if (abs((*encoders_pos_)(i)-future_ref(i)) > max_joint_position_track_error_ + max_joint_position_variation_)
            {
                yError() << module_name_ + "::updateModule(). Error: Low joint position tracking accuracy. At least one future joint position reference differs more than "+ std::to_string(max_joint_position_variation_ + max_joint_position_track_error_)+" w.r.t. the related joint position measured value!";
                error_prev_state_ = State::Running;
                setState(State::Error);
                return false;
            }
        }

        if (!cub_joint_control_.moveToStreaming(future_ref))
        {
            yError() << module_name_ + "::updateModule(). Error: Cannot set desired joint position reference. See the errors above.";
            error_prev_state_ = State::Running;
            setState(State::Error);
            return false;
        }

        if(getDuration() == 0.0 || (traj_.lin_gen->isEnd() && traj_.ang_gen->isEnd())){
            bool is_ended = true;
            auto ik_joint_vel_abs = ik_joint_vel_.array().abs();

            for (int i = 0; i < ik_joint_vel_abs.size(); ++i)
            {
                if (ik_joint_vel_abs(i) > stop_vel_)
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
        double position_error = (getTrajFinPose().translation() - traj_.init_pose.translation()).norm();

        while (!(traj_.lin_gen->isEnd() && traj_.ang_gen->isEnd()) && (iteration <= max_iter_) && (position_error > pos_err_th_))
        {
            if(!updateReferenceVelocities())
            {
                yError() << module_name_ + "::updateModule(). See error(s) above.";
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

            iteration++;
            position_error = (getTrajFinPose().translation() - desired_transform_.translation()).norm();
        }

        if (module_verbose_)
        {
            yInfo() << "position_error | pos_err_th_\t" << position_error << "|" << pos_err_th_;
            yInfo() << "iteration | max_iter_\t" << iteration << "|" << max_iter_;
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
    //-------------------------------------------
    //  Quick flags overview
    //-------------------------------------------
    yInfo() << module_name_ << "::updateModule()  "
        << "module_logging=" << std::boolalpha << module_logging_
        << ", module_verbose=" << std::boolalpha << module_verbose_;

    if (module_logging_ || module_verbose_)
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

bool Module::updateReferenceVelocities()
{
    /* Get the previous state of the integrator */
    const Eigen::VectorXd q_prev = vel2pos_integrator_->get_state();

    /* Get the previous input of the integrator */
    const Eigen::VectorXd dq_prev = acc2vel_integrator_->get_state();

    /* Set joints and update fk_-> */
    fk_->set_joints_state(q_prev, dq_prev, *ik_joint_acc_);
    fk_->update();

    /* Get forward kinematics. */
    auto transform = fk_->get_ee_transform();

    /* Get jacobian. */
    auto jacobian = fk_->get_jacobian();

    /* Get velocity. */
    auto frameBiasAcc = fk_->get_ee_bias_acc();

    /* Set robot state in the ik_-> */
    ik_->set_robot_state(q_prev, dq_prev, transform, jacobian, frameBiasAcc);

    /* Evaluate trajectory */
    if(traj_.duration != 0.0){
        traj_.lin_gen->step();
        traj_.ang_gen->step();
        desired_transform_ = Eigen::Translation3d(traj_.lin_gen->getPosition());
        desired_transform_.rotate(traj_.ang_gen->getOrientation());
        desired_lin_vel_ = traj_.lin_gen->getVelocity();
        desired_ang_vel_ = traj_.ang_gen->getVelocity();
    }
    else{
        desired_transform_ = getTrajFinPose();
        desired_lin_vel_ = Eigen::Vector3d::Zero();
        desired_ang_vel_ = Eigen::Vector3d::Zero();
    }


    /* Set desired transform in the ik_-> */
    ik_->set_desired_ee_transform(desired_transform_);
    ik_->set_desired_ee_twist(desired_lin_vel_, desired_ang_vel_);

    /* Solve the ik_-> and retrieve solution*/
    ik_joint_acc_ = ik_->eval_reference_velocities();

    if (!ik_joint_acc_.has_value())
    {
        yError() << module_name_ + "::updateReferenceVelocities(). Error: No value for the ik solution!";
        return true;
    }

    acc2vel_integrator_->integrate(*ik_joint_acc_);
    ik_joint_vel_ = acc2vel_integrator_->get_state();

    return true;
}

bool Module::checkInputAndUpdateIntegrator()
{
    /* Safety Check: QP output doesn't cause an abrupt movement. */
    auto ik_joint_pos_var = sample_time_ * ik_joint_vel_.array().abs();
    for (int i = 0; i < ik_joint_pos_var.size(); ++i)
    {
        if (ik_joint_pos_var(i) > max_joint_position_variation_)
        {
            return false;
        }
    }

    /* Evaluate the integral of the reference velocities. */
    vel2pos_integrator_->integrate(ik_joint_vel_);

    return true;
}

void Module::log()
{
    auto state = getState();

    //Trajectory values
    Eigen::VectorXd traj_ini_pos = traj_.init_pose.translation();
    Eigen::Matrix3d traj_ini_rot = traj_.init_pose.rotation();
    Eigen::VectorXd traj_fin_pos = getTrajFinPose().translation();
    Eigen::Matrix3d traj_fin_rot = getTrajFinPose().rotation();
    Eigen::VectorXd traj_des_pos = desired_transform_.translation();
    Eigen::Matrix3d traj_des_rot = desired_transform_.rotation();
    Eigen::AngleAxisd traj_des_rot_aa(traj_des_rot);
    Eigen::Quaterniond traj_des_rot_quat(traj_des_rot);

    //QP generated values
    Eigen::VectorXd qp_joints_acc = *ik_joint_acc_;
    Eigen::VectorXd qp_joints_vel = ik_joint_vel_;
    Eigen::VectorXd qp_joints_pos = vel2pos_integrator_->get_state();
    fk_->set_joints_state(qp_joints_pos, qp_joints_vel, qp_joints_acc);
    fk_->update();
    auto qp_trf = fk_->get_ee_transform();
    Eigen::VectorXd qp_pos = qp_trf.translation();
    Eigen::Matrix3d qp_rot = qp_trf.rotation();
    Eigen::AngleAxisd qp_rot_aa (qp_rot);
    Eigen::Quaterniond qp_rot_quat(qp_rot_aa);

    auto qp_jacobian = fk_->get_jacobian();
    double qp_manip = sqrt((qp_jacobian * qp_jacobian.transpose()).determinant());
    static double qp_max_manip = 0.0;
    if (qp_max_manip < qp_manip) qp_max_manip = qp_manip;
    double qp_weight_manip_function =  pow(1 - (qp_manip / qp_max_manip), 2);

    Eigen::VectorXd qp_e_pos = traj_des_pos - qp_pos;
    Eigen::AngleAxisd qp_e_rot_aa(desired_transform_.rotation() * qp_rot.transpose());
    Eigen::Vector3d qp_e_rot = qp_e_rot_aa.axis() * qp_e_rot_aa.angle();

    //Measured values
    Eigen::VectorXd meas_enc_pos = *encoders_pos_;
    Eigen::VectorXd meas_enc_vel = *encoders_vel_;
    Eigen::VectorXd meas_enc_acc = *encoders_acc_;
    fk_->set_joints_state(meas_enc_pos, meas_enc_vel, meas_enc_acc);
    fk_->update();
    Eigen::Affine3d meas_trf = fk_->get_ee_transform();
    Eigen::VectorXd meas_pos = meas_trf.translation();
    Eigen::Matrix3d meas_rot = meas_trf.rotation();
    Eigen::AngleAxisd meas_rot_aa (meas_rot);
    Eigen::Quaterniond meas_rot_quat(meas_rot_aa);
    Eigen::VectorXd meas_ee_bias_acc = fk_->get_ee_bias_acc();

    Eigen::VectorXd meas_e_pos = qp_pos - meas_pos;
    Eigen::AngleAxisd meas_e_rot_aa(qp_rot * meas_rot.transpose());
    Eigen::Vector3d meas_e_rot = meas_e_rot_aa.axis() * meas_e_rot_aa.angle();

    auto meas_jacobian = fk_->get_jacobian();
    double meas_manip = sqrt((meas_jacobian * meas_jacobian.transpose()).determinant());
    static double meas_max_manip = 0.0;
    if (meas_max_manip < meas_manip) meas_max_manip = meas_manip;
    double meas_weight_manip_function =  pow(1 - (meas_manip / meas_max_manip), 2);

    if (module_verbose_)
    {
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
        yInfo() << "---------- Desired min-jerk trajectory --------------------------";
        yInfo() << "pos des" << eigenToString(traj_des_pos);
        yInfo() << "ori des" << eigenToString(traj_des_rot_aa.axis() * traj_des_rot_aa.angle());
        yInfo() <<" ori qua: "<<traj_des_rot_quat.x()<<" "<<traj_des_rot_quat.y()<<" "<<traj_des_rot_quat.z()<<" "<<traj_des_rot_quat.w();

        //QP generated values
        yInfo() << "---------- QP generated values, errors w.r.t. min-jerk ----------";
        yInfo() << "pos gen" << eigenToString(qp_pos);
        yInfo() << "ori gen" << eigenToString(qp_rot_aa.axis() * qp_rot_aa.angle());
        yInfo() <<" ori qua: "<<qp_rot_quat.x()<<" "<<qp_rot_quat.y()<<" "<<qp_rot_quat.z()<<" "<<qp_rot_quat.w();
        yInfo() << "pos err (m) |norm| [components]" << qp_e_pos.norm() << "\t" << eigenToString(qp_e_pos);
        yInfo() << "ang err (deg) |norm| [components]" << qp_e_rot_aa.angle() * (180 / M_PI) << "\t" << eigenToString(qp_e_rot);
        yInfo() << "joints qp acc" << eigenToString(qp_joints_acc);
        yInfo() << "joints qp vel" << eigenToString(qp_joints_vel);
        yInfo() << "joints qp pos" << eigenToString(qp_joints_pos);
        yInfo() << "manip "<<qp_manip<<" max_manip "<<qp_max_manip<<" manip/max_manip "<<qp_manip/qp_max_manip<<" weight_manip_function "<<qp_weight_manip_function;

        //measured values
        yInfo() << "---------- Current values, errors w.r.t. QP generated -----------";
        yInfo() << "pos cur" << eigenToString(meas_pos);
        yInfo() << "ori cur" << eigenToString(meas_rot_aa.axis() * meas_rot_aa.angle());
        yInfo() <<" ori qua: "<<meas_rot_quat.x()<<" "<<meas_rot_quat.y()<<" "<<meas_rot_quat.z()<<" "<<meas_rot_quat.w();
        yInfo() << "pos err (m) |norm| [components]" << meas_e_pos.norm() << "\t" << eigenToString(meas_e_pos);
        yInfo() << "ang err (deg) |norm| [components]" << meas_e_rot_aa.angle() * (180 / M_PI) << "\t" << eigenToString(meas_e_rot);
        yInfo() << "joints meas pos" << eigenToString(meas_enc_pos);
        yInfo() << "joints meas vel" << eigenToString(meas_enc_vel);
        yInfo() << "joints meas acc" << eigenToString(meas_enc_acc);
        yInfo() << "joints err  pos" << eigenToString(qp_joints_pos - meas_enc_pos);
        yInfo() << "joints err  vel" << eigenToString(qp_joints_vel - meas_enc_vel);
        yInfo() << "joints err  acc" << eigenToString(qp_joints_acc - meas_enc_acc);
        yInfo() << "################################################################";
    }

    if(module_logging_)
    {

        Eigen::VectorXd vec3(3);

        m_vectorsCollectionServer.prepareData();
        m_vectorsCollectionServer.clearData();

        m_vectorsCollectionServer.populateData("cartesian_state", std::array<double, 1>{static_cast<double>(getState())});

        auto eigenToStdVecDouble = [] (const Eigen::VectorXd& eigen_vec)
        {
            std::vector<double> vec(eigen_vec.size());

            for (int i = 0; i < eigen_vec.size(); i++)
            {
                vec[i] = eigen_vec[i];
            }

            return vec;
        };

        m_vectorsCollectionServer.populateData("trajGen::pos::ini", eigenToStdVecDouble(traj_ini_pos));
        m_vectorsCollectionServer.populateData("trajGen::pos::fin", eigenToStdVecDouble(traj_fin_pos));
        m_vectorsCollectionServer.populateData("trajGen::pos::des", eigenToStdVecDouble(traj_des_pos));

        //eulerAngles(2, 1, 0) -> Rz(yaw) * Ry(pitch) * Rx(roll) (https://github.com/robotology/idyntree/issues/721#issue-674991575)
        m_vectorsCollectionServer.populateData("trajGen::ori::ini", eigenToStdVecDouble(traj_ini_rot.eulerAngles(2, 1, 0)));
        m_vectorsCollectionServer.populateData("trajGen::ori::fin", eigenToStdVecDouble(traj_fin_rot.eulerAngles(2, 1, 0)));
        m_vectorsCollectionServer.populateData("trajGen::ori::des", eigenToStdVecDouble(traj_des_rot.eulerAngles(2, 1, 0)));


        m_vectorsCollectionServer.populateData("qpIK::joints::pos", eigenToStdVecDouble(qp_joints_pos));
        m_vectorsCollectionServer.populateData("qpIK::joints::vel", eigenToStdVecDouble(qp_joints_vel));
        m_vectorsCollectionServer.populateData("qpIK::joints::acc", eigenToStdVecDouble(qp_joints_acc));
        m_vectorsCollectionServer.populateData("qpIK::cartes::pos", eigenToStdVecDouble(qp_pos));

        m_vectorsCollectionServer.populateData("qpIK::cartes::ori", eigenToStdVecDouble(qp_rot.eulerAngles(2, 1, 0)));

        vec3[0]=qp_manip;
        vec3[1]=qp_max_manip;
        vec3[2]=qp_weight_manip_function;
        m_vectorsCollectionServer.populateData("qpIK::manips", eigenToStdVecDouble(vec3));


        m_vectorsCollectionServer.populateData("measured::joints::pos", eigenToStdVecDouble(meas_enc_pos));
        m_vectorsCollectionServer.populateData("measured::joints::vel", eigenToStdVecDouble(meas_enc_vel));
        m_vectorsCollectionServer.populateData("measured::joints::acc", eigenToStdVecDouble(meas_enc_acc));
        m_vectorsCollectionServer.populateData("measured::cartes::pos", eigenToStdVecDouble(meas_pos));

        m_vectorsCollectionServer.populateData("measured::cartes::ori", eigenToStdVecDouble( meas_rot.eulerAngles(2, 1, 0)));
        vec3[0]=meas_manip;
        vec3[1]=meas_max_manip;
        vec3[2]=meas_weight_manip_function;
        m_vectorsCollectionServer.populateData("measured::manips", eigenToStdVecDouble(vec3));

        m_vectorsCollectionServer.sendData();

    }

}
