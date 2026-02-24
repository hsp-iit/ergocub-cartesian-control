#include <module.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>
#include <yarp/eigen/Eigen.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    auto groupCheckAndRetrieve = [&](const yarp::os::ResourceFinder &rf_in, const std::string &group_name_in, yarp::os::Bottle &bottle_group_out)
    {
        if (!rf_in.check(group_name_in))
        {
            yError() << "[" + module_name_ + "::configure] Error: mandatory group " + group_name_in + " missing.";
            return false;
        }

        bottle_group_out = rf_in.findGroup(group_name_in);

        return true;
    };


    /* general.ini */
    yarp::os::Bottle COMMON_bot;
    if (!groupCheckAndRetrieve(rf, "COMMON", COMMON_bot))
        return false;

    if  (   !(utils::checkParameters({{"rate"}}, "", COMMON_bot, "", utils::ParameterType::Float64, false))
        ||  !(utils::checkParameters({{"module_logging", "module_verbose", "qp_verbose"}}, "", COMMON_bot, "", utils::ParameterType::Bool, false))
        ||  !(utils::checkParameters({{"rpc_local_port_name", "ctrl_local_port_name"}}, "", COMMON_bot, "", utils::ParameterType::String, false)))
    {
        yError() << "[" + module_name_ + "::configure] Error: mandatory parameter(s) for COMMON group missing or invalid.";
        return false;
    }

    sample_time_ = 1.0 / COMMON_bot.find("rate").asFloat64();
    module_logging_ = COMMON_bot.find("module_logging").asBool();
    module_verbose_ = COMMON_bot.find("module_verbose").asBool();
    const bool qp_verbose = COMMON_bot.find("qp_verbose").asBool();
    const std::string rpc_local_port_name = COMMON_bot.find("rpc_local_port_name").asString();
    bp_cmd_port_.open(COMMON_bot.find("ctrl_local_port_name").asString());
    no_control_ = COMMON_bot.check("no_control") ? COMMON_bot.find("no_control").asBool() : false;
    if (no_control_)
    {
        joints_pos_port_.open(("/" + module_name_ + "/joints_pos:o").c_str());
    }

    /* Enabling of the chains is optional */
    right_enabled_ = rf.check("RIGHT_ARM");
    left_enabled_  = rf.check("LEFT_ARM");
    torso_enabled_ = rf.check("TORSO");

    if (!right_enabled_ && !left_enabled_ && !torso_enabled_)
    {
        yError() << "[" + module_name_ + "::configure] At least one chain must be enabled (RIGHT_ARM, LEFT_ARM, TORSO).";
        return false;
    }

    /* right_arm.ini (optional) */
    yarp::os::Bottle RIGHT_ARM_bot;
    if (right_enabled_)
    {
        if (!groupCheckAndRetrieve(rf, "RIGHT_ARM", RIGHT_ARM_bot))
            return false;

        if  (   !(utils::checkParameters({{"improve_manip_weight", "joint_limits_param", "joint_acc_weight", "cartesian_pos_weight", "cartesian_pos_p_gain", "cartesian_pos_d_gain", "cartesian_ori_weight", "cartesian_ori_p_gain", "cartesian_ori_d_gain", "stop_vel"}}, "", RIGHT_ARM_bot, "", utils::ParameterType::Float64, false))
            ||  !(utils::checkParameters({{"joint_pos_weight", "joint_pos_p_gain", "joint_pos_d_gain"}}, "", RIGHT_ARM_bot, "", utils::ParameterType::Float64, true))
            ||  !(utils::checkParameters({{"root_frame_name", "ee_frame_name"}}, "", RIGHT_ARM_bot, "", utils::ParameterType::String, false)))
        {
            yError() << "[" + module_name_ + "::configure] Error: mandatory parameter(s) for RIGHT_ARM group missing or invalid.";
            return false;
        }
    }

    /* left_arm.ini (optional) */
    yarp::os::Bottle LEFT_ARM_bot;
    if (left_enabled_)
    {
        if (!groupCheckAndRetrieve(rf, "LEFT_ARM", LEFT_ARM_bot))
            return false;

        if  (   !(utils::checkParameters({{"improve_manip_weight", "joint_limits_param", "joint_acc_weight", "cartesian_pos_weight", "cartesian_pos_p_gain", "cartesian_pos_d_gain", "cartesian_ori_weight", "cartesian_ori_p_gain", "cartesian_ori_d_gain", "stop_vel"}}, "", LEFT_ARM_bot, "", utils::ParameterType::Float64, false))
            ||  !(utils::checkParameters({{"joint_pos_weight", "joint_pos_p_gain", "joint_pos_d_gain"}}, "", LEFT_ARM_bot, "", utils::ParameterType::Float64, true))
            ||  !(utils::checkParameters({{"root_frame_name", "ee_frame_name"}}, "", LEFT_ARM_bot, "", utils::ParameterType::String, false)))
        {
            yError() << "[" + module_name_ + "::configure] Error: mandatory parameter(s) for LEFT_ARM group missing or invalid.";
            return false;
        }
    }

    /* torso.ini (optional) */
    yarp::os::Bottle TORSO_bot;
    if (torso_enabled_)
    {
        if (!groupCheckAndRetrieve(rf, "TORSO", TORSO_bot))
            return false;

        if  (   !(utils::checkParameters({{"joint_limits_param", "joint_acc_weight"}}, "", TORSO_bot, "", utils::ParameterType::Float64, false))
            ||  !(utils::checkParameters({{"joint_pos_weight", "joint_pos_p_gain", "joint_pos_d_gain"}}, "", TORSO_bot, "", utils::ParameterType::Float64, true)))
        {
            yError() << "[" + module_name_ + "::configure] Error: mandatory parameter(s) for TORSO group missing or invalid.";
            return false;
        }
    }

    /* Configure RPC service. */
    if (!configureService(rf, rpc_local_port_name))
    {
        yError() << "[" + module_name_ + "::configure] Error: cannot configure the RPC service.";
        return false;
    }

    /* Instantiate and initialize CHAINS and SUBCHAINS. */
    {
        if (right_enabled_)
        {
            if (!right_arm_.cjc.configure(RIGHT_ARM_bot))
            {
                yError() << "[" + module_name_ + "::configure] Error: cannot configure the cubJointControl instance for RIGHT_ARM. See the errors above.";
                return false;
            }
            if (!right_arm_.cjc.configureJointsMode("Streaming"))
            {
                yError() << "[" + module_name_ + "::configure] Error: Cannot set 'Streaming' control mode for RIGHT_ARM. See the errors above.";
                return false;
            }
        }

        if (left_enabled_)
        {
            if (!left_arm_.cjc.configure(LEFT_ARM_bot))
            {
                yError() << "[" + module_name_ + "::configure] Error: cannot configure the cubJointControl instance for LEFT_ARM. See the errors above.";
                return false;
            }
            if (!left_arm_.cjc.configureJointsMode("Streaming"))
            {
                yError() << "[" + module_name_ + "::configure] Error: Cannot set 'Streaming' control mode for LEFT_ARM. See the errors above.";
                return false;
            }
        }

        if (torso_enabled_)
        {
            if (!torso_.cjc.configure(TORSO_bot))
            {
                yError() << "[" + module_name_ + "::configure] Error: cannot configure the cubJointControl instance for TORSO. See the errors above.";
                return false;
            }
            if (!torso_.cjc.configureJointsMode("Streaming"))
            {
                yError() << "[" + module_name_ + "::configure] Error: Cannot set 'Streaming' control mode for TORSO. See the errors above.";
                return false;
            }
        }

        if(!encodersMeasUpdate())
        {
            yError() << "[" + module_name_ + "::configure] Error: cannot read encoders. See errors above.";
            return false;
        }
    }

    /* Instantiate and initialize iDynTree-based forward kinematics. */
    {
        /* Get robot urdf path. */
        const std::string robot_urdf_path = rf.findFileByName("model.urdf");
        if (robot_urdf_path.empty())
        {
            yError() << "[" + module_name_ + "::configure] Error: cannot load the robot urdf path. Please chack that the YARP_ROBOT_NAME environment variable is set.";
            return false;
        }

        std::vector<std::string> torso_joints_list;

        if (torso_enabled_)
        {
            torso_joints_list = utils::loadVectorString(TORSO_bot, "joint_axes_list");
        }

        /* right arm*/
        if (right_enabled_)
        {
            auto right_chain_joints_list = utils::loadVectorString(RIGHT_ARM_bot, "joint_axes_list");
            if (torso_enabled_)
                right_chain_joints_list.insert(right_chain_joints_list.end(), torso_joints_list.begin(), torso_joints_list.end());

            right_chain_.measFk = std::make_unique<ForwardKinematicsiDynTree>(robot_urdf_path, right_chain_joints_list, RIGHT_ARM_bot.find("root_frame_name").asString(), RIGHT_ARM_bot.find("ee_frame_name").asString());
            right_chain_.refFk  = std::make_unique<ForwardKinematicsiDynTree>(robot_urdf_path, right_chain_joints_list, RIGHT_ARM_bot.find("root_frame_name").asString(), RIGHT_ARM_bot.find("ee_frame_name").asString());
        }

        /* left arm*/
        if (left_enabled_)
        {
            auto left_chain_joints_list = utils::loadVectorString(LEFT_ARM_bot, "joint_axes_list");
            if (torso_enabled_)
                left_chain_joints_list.insert(left_chain_joints_list.end(), torso_joints_list.begin(), torso_joints_list.end());

            left_chain_.measFk = std::make_unique<ForwardKinematicsiDynTree>(robot_urdf_path, left_chain_joints_list, LEFT_ARM_bot.find("root_frame_name").asString(), LEFT_ARM_bot.find("ee_frame_name").asString());
            left_chain_.refFk  = std::make_unique<ForwardKinematicsiDynTree>(robot_urdf_path, left_chain_joints_list, LEFT_ARM_bot.find("root_frame_name").asString(), LEFT_ARM_bot.find("ee_frame_name").asString());
        }
    }

    /* Instantiate and initialize QP_inverse kinematics. */
    {
        Eigen::VectorXd joint_limits_params, joint_acc_weights;
        Eigen::VectorXd joint_pos_weights, joint_pos_p_gain, joint_pos_d_gain;
        Eigen::VectorXd cartesian_pos_weight, cartesian_pos_p_gain, cartesian_pos_d_gain;
        Eigen::VectorXd cartesian_ori_weight, cartesian_ori_p_gain, cartesian_ori_d_gain;
        Eigen::VectorXd joint_home;

        auto appendOrInit = [&](Eigen::VectorXd &dst, const Eigen::VectorXd &src)
        {
            Eigen::VectorXd temp(dst);
            dst.resize(temp.size() + src.size());
            if (temp.size() > 0) dst << temp, src; else dst << src;
        };

        // RIGHT ARM (joint e cartesian)
        if (right_enabled_)
        {
            appendOrInit(joint_limits_params, Eigen::Vector<double,1>(RIGHT_ARM_bot.find("joint_limits_param").asFloat64()));
            appendOrInit(joint_acc_weights,   Eigen::Vector<double,1>(RIGHT_ARM_bot.find("joint_acc_weight").asFloat64()));

            appendOrInit(joint_pos_weights, utils::loadVectorDouble(RIGHT_ARM_bot, "joint_pos_weight"));
            appendOrInit(joint_pos_p_gain,  utils::loadVectorDouble(RIGHT_ARM_bot, "joint_pos_p_gain"));
            appendOrInit(joint_pos_d_gain,  utils::loadVectorDouble(RIGHT_ARM_bot, "joint_pos_d_gain"));

            appendOrInit(joint_home, *right_arm_.joint_pos);
        }
        // LEFT ARM (joint e cartesian)
        if (left_enabled_)
        {
            appendOrInit(joint_limits_params, Eigen::Vector<double,1>(LEFT_ARM_bot.find("joint_limits_param").asFloat64()));
            appendOrInit(joint_acc_weights,   Eigen::Vector<double,1>(LEFT_ARM_bot.find("joint_acc_weight").asFloat64()));

            appendOrInit(joint_pos_weights, utils::loadVectorDouble(LEFT_ARM_bot, "joint_pos_weight"));
            appendOrInit(joint_pos_p_gain,  utils::loadVectorDouble(LEFT_ARM_bot, "joint_pos_p_gain"));
            appendOrInit(joint_pos_d_gain,  utils::loadVectorDouble(LEFT_ARM_bot, "joint_pos_d_gain"));

            appendOrInit(joint_home, *left_arm_.joint_pos);
        }
        // TORSO (only joint terms)
        if (torso_enabled_)
        {
            appendOrInit(joint_limits_params, Eigen::Vector<double,1>(TORSO_bot.find("joint_limits_param").asFloat64()));
            appendOrInit(joint_acc_weights,   Eigen::Vector<double,1>(TORSO_bot.find("joint_acc_weight").asFloat64()));

            appendOrInit(joint_pos_weights, utils::loadVectorDouble(TORSO_bot, "joint_pos_weight"));
            appendOrInit(joint_pos_p_gain,  utils::loadVectorDouble(TORSO_bot, "joint_pos_p_gain"));
            appendOrInit(joint_pos_d_gain,  utils::loadVectorDouble(TORSO_bot, "joint_pos_d_gain"));

            appendOrInit(joint_home, *torso_.joint_pos);
        }

        // Cartesian weights: the dimension is fixed to 2 (Right and Left). If one EE is not enabled --> weight/gain = 0.
        auto s = [](double v) { Eigen::Vector<double,1> x; x << v; return x; };

        if (right_enabled_)
        {
            appendOrInit(cartesian_pos_weight, s(RIGHT_ARM_bot.find("cartesian_pos_weight").asFloat64()));
            appendOrInit(cartesian_pos_p_gain, s(RIGHT_ARM_bot.find("cartesian_pos_p_gain").asFloat64()));
            appendOrInit(cartesian_pos_d_gain, s(RIGHT_ARM_bot.find("cartesian_pos_d_gain").asFloat64()));

            appendOrInit(cartesian_ori_weight, s(RIGHT_ARM_bot.find("cartesian_ori_weight").asFloat64()));
            appendOrInit(cartesian_ori_p_gain, s(RIGHT_ARM_bot.find("cartesian_ori_p_gain").asFloat64()));
            appendOrInit(cartesian_ori_d_gain, s(RIGHT_ARM_bot.find("cartesian_ori_d_gain").asFloat64()));
        }
        else
        {
            appendOrInit(cartesian_pos_weight, s(0.0));
            appendOrInit(cartesian_pos_p_gain, s(0.0));
            appendOrInit(cartesian_pos_d_gain, s(0.0));

            appendOrInit(cartesian_ori_weight, s(0.0));
            appendOrInit(cartesian_ori_p_gain, s(0.0));
            appendOrInit(cartesian_ori_d_gain, s(0.0));
        }

        if (left_enabled_)
        {
            appendOrInit(cartesian_pos_weight, s(LEFT_ARM_bot.find("cartesian_pos_weight").asFloat64()));
            appendOrInit(cartesian_pos_p_gain, s(LEFT_ARM_bot.find("cartesian_pos_p_gain").asFloat64()));
            appendOrInit(cartesian_pos_d_gain, s(LEFT_ARM_bot.find("cartesian_pos_d_gain").asFloat64()));

            appendOrInit(cartesian_ori_weight, s(LEFT_ARM_bot.find("cartesian_ori_weight").asFloat64()));
            appendOrInit(cartesian_ori_p_gain, s(LEFT_ARM_bot.find("cartesian_ori_p_gain").asFloat64()));
            appendOrInit(cartesian_ori_d_gain, s(LEFT_ARM_bot.find("cartesian_ori_d_gain").asFloat64()));
        }
        else
        {
            appendOrInit(cartesian_pos_weight, s(0.0));
            appendOrInit(cartesian_pos_p_gain, s(0.0));
            appendOrInit(cartesian_pos_d_gain, s(0.0));

            appendOrInit(cartesian_ori_weight, s(0.0));
            appendOrInit(cartesian_ori_p_gain, s(0.0));
            appendOrInit(cartesian_ori_d_gain, s(0.0));
        }

        // RIGHT AND LEFT ARM: stop_vel & improve_manip_weight
        std::vector<double> stop_vels, manip_ws;
        if (right_enabled_) { stop_vels.push_back(RIGHT_ARM_bot.find("stop_vel").asFloat64());
                              manip_ws.push_back(RIGHT_ARM_bot.find("improve_manip_weight").asFloat64()); }
        if (left_enabled_)  { stop_vels.push_back(LEFT_ARM_bot.find("stop_vel").asFloat64());
                              manip_ws.push_back(LEFT_ARM_bot.find("improve_manip_weight").asFloat64()); }

        compound_chain_.stop_vel = stop_vels.empty() ? 1e-3 : *std::min_element(stop_vels.begin(), stop_vels.end());

        // If manip_ws is empty --> improve_manip_weight = 0.0, otherwise compute the minimum value
        double improve_manip_weight = manip_ws.empty() ? 0.0 : *std::min_element(manip_ws.begin(), manip_ws.end());
        
        // If enabled compute the number of joints for each chain otherwise set to 0
        int nR = right_enabled_ ? right_arm_.cjc.getNumberJoints() : 0;
        int nL = left_enabled_  ? left_arm_.cjc.getNumberJoints()  : 0;
        int nT = torso_enabled_ ? torso_.cjc.getNumberJoints()     : 0;

        // IK
        try{
            compound_chain_.ik = std::make_unique<DifferentialInverseKinematicsQP>(sample_time_,
                                                                                qp_verbose,
                                                                                nR,
                                                                                nL,
                                                                                nT,
                                                                                joint_acc_weights,
                                                                                joint_pos_weights,
                                                                                joint_pos_p_gain,
                                                                                joint_pos_d_gain,
                                                                                cartesian_pos_weight,
                                                                                cartesian_pos_p_gain,
                                                                                cartesian_pos_d_gain,
                                                                                cartesian_ori_weight,
                                                                                cartesian_ori_p_gain,
                                                                                cartesian_ori_d_gain,
                                                                                improve_manip_weight,
                                                                                joint_home);
        }
        catch( const std::runtime_error& e ) {
            yError() << e.what();
            yError() << "[" + module_name_ + "::configure] See error(s) above.";
            return false;
        }

        // Retrieve joint limits and set in IK
        Eigen::VectorXd lower_limits, upper_limits;

        if (right_enabled_)
        {
            auto r_joints_limit = right_arm_.cjc.getJointLimits();
            if (!r_joints_limit.has_value())
            {
                yError() << "[" + module_name_ + "::configure]Error: CubJointControl cannot retrieve RIGHT_ARM joint limits.";
                return false;
            }
            appendEigen(lower_limits, (*r_joints_limit)["lower"]);
            appendEigen(upper_limits, (*r_joints_limit)["upper"]);
        }
        if (left_enabled_)
        {
            auto l_joints_limit = left_arm_.cjc.getJointLimits();
            if (!l_joints_limit.has_value())
            {
                yError() << "[" + module_name_ + "::configure]Error: CubJointControl cannot retrieve LEFT_ARM joint limits.";
                return false;
            }
            appendEigen(lower_limits, (*l_joints_limit)["lower"]);
            appendEigen(upper_limits, (*l_joints_limit)["upper"]);
        }
        if (torso_enabled_)
        {
            auto t_joints_limit = torso_.cjc.getJointLimits();
            if (!t_joints_limit.has_value())
            {
                yError() << "[" + module_name_ + "::configure]Error: CubJointControl cannot retrieve TORSO joint limits.";
                return false;
            }
            appendEigen(lower_limits, (*t_joints_limit)["lower"]);
            appendEigen(upper_limits, (*t_joints_limit)["upper"]);
        }

        if(!compound_chain_.ik->set_joint_limits(lower_limits, upper_limits, joint_limits_params))
        {
            yError() << "[" + module_name_ + "::configure] See error(s) above.";
            return false;
        }

        /* Instantiate and initialize integrators */
        Eigen::VectorXd init_pos, init_vel, init_acc;
        if (right_enabled_) { appendEigen(init_pos, *right_arm_.joint_pos); appendEigen(init_vel, *right_arm_.joint_vel); appendEigen(init_acc, *right_arm_.joint_acc); }
        if (left_enabled_)  { appendEigen(init_pos, *left_arm_.joint_pos);  appendEigen(init_vel, *left_arm_.joint_vel);  appendEigen(init_acc, *left_arm_.joint_acc); }
        if (torso_enabled_) { appendEigen(init_pos, *torso_.joint_pos);     appendEigen(init_vel, *torso_.joint_vel);     appendEigen(init_acc, *torso_.joint_acc); }

        compound_chain_.vel2pos_integrator = std::make_unique<Integrator>(sample_time_);
        compound_chain_.vel2pos_integrator->set_initial_condition(init_pos);

        compound_chain_.acc2vel_integrator = std::make_unique<Integrator>(sample_time_);
        compound_chain_.acc2vel_integrator->set_initial_condition(init_vel);

        /* Initialize joint refs */
        qp_result_ = init_acc;
        encodersRefUpdate();
    }

    measFksUpdate();

    if (right_enabled_)
    {
        right_chain_.home_pose = right_chain_.measFk->get_ee_transform();
        right_desired_pose_ = right_chain_.home_pose;
    }
    else
    {
        right_desired_pose_.setIdentity();
    }

    if (left_enabled_)
    {
        left_chain_.home_pose = left_chain_.measFk->get_ee_transform();
        left_desired_pose_ = left_chain_.home_pose;
    }
    else
    {
        left_desired_pose_.setIdentity();
    }

    right_desired_lin_vel_ = Eigen::Vector3d::Zero();
    right_desired_ang_vel_ = Eigen::Vector3d::Zero();
    right_desired_lin_acc_ = Eigen::Vector3d::Zero();
    right_desired_ang_acc_ = Eigen::Vector3d::Zero();

    left_desired_lin_vel_ = Eigen::Vector3d::Zero();
    left_desired_ang_vel_ = Eigen::Vector3d::Zero();
    left_desired_lin_acc_ = Eigen::Vector3d::Zero();
    left_desired_ang_acc_ = Eigen::Vector3d::Zero();

    setState(State::Stop);

    yInfo() << "[" + module_name_ + "::configure] Done.";

    return true;
}

bool Module::close()
{
    rpc_cmd_port_.close();
    bp_cmd_port_.close();
    joints_pos_port_.close();

    return true;
}

double Module::getPeriod()
{
    return sample_time_;
}

bool Module::interruptModule()
{
    rpc_cmd_port_.interrupt();
    bp_cmd_port_.interrupt();
    joints_pos_port_.interrupt();

    return true;
}

bool Module::updateModule()
{
    checkAndReadRpcCommands();

    {
        constexpr int max_retries = 5;
        constexpr auto retry_delay = std::chrono::milliseconds(10);
        int attempt = 0;
        bool success = false;
        while (attempt < max_retries) {
            if (encodersMeasUpdate()) {
                success = true;
                break;
            }
            std::this_thread::sleep_for(retry_delay);
            ++attempt;
        }
        if (!success) {
            yError()<< "[" + module_name_ + "::updateModule] See error(s) above after " << max_retries << " attempts.";
            return false;
        }
    }

    measFksUpdate();

    if(checkAndReadNewInputs())
    {
        setState(State::Running);
    }

    if(getState()==State::Running)
    {
        yDebug() << "[" + module_name_ + "::updateModule] DENTRO IF RUNNING.";
        refFksUpdate();
        
        ikUpdate();
        setDesiredTrajectory();

        if(!solveIkAndUpdateIntegrators())
        {
            yError()<< "[" + module_name_ + "::updateModule] See error(s) above.";
            return false;
        }
        
        if (!encodersRefUpdate())
        {
            yError()<< "[" + module_name_ + "::updateModule] See error(s) above.";
            return false;
        }
        if (!no_control_)
        {
            if(!moveChains())
            {
                yError()<< "[" + module_name_ + "::updateModule] See error(s) above.";
                return false;
            }
        }
        if(isMotionDone())
        {
            compound_chain_.vel2pos_integrator->set_initial_condition(compound_chain_.joints.pos);
            compound_chain_.acc2vel_integrator->set_initial_condition(Eigen::VectorXd::Zero(compound_chain_.joints.pos.size()));
            setState(State::Stop);
        }

    }

    if (no_control_)
    {
        yarp::sig::Vector& out = joints_pos_port_.prepare();
        out.resize(compound_chain_.joints.pos.size());
        for (size_t i = 0; i < out.size(); ++i)
            out[i] = compound_chain_.joints.pos[i];
        joints_pos_port_.write();
    }
    

    if (module_logging_ || module_verbose_)
        verboseAndLog();

    return true;
}


bool Module::encodersMeasUpdate()
{
    // Read only the enabled chains
    if (right_enabled_)
    {
        right_arm_.joint_pos = right_arm_.cjc.getJointValues();
        right_arm_.joint_vel = right_arm_.cjc.getJointSpeeds();
        right_arm_.joint_acc = right_arm_.cjc.getJointAccelerations();

        if (!right_arm_.joint_pos.has_value())
        {
            yError() << "[" + module_name_ + "::encodersMeasUpdate] Error: CubJointControl cannot retrieve RIGHT_ARM joint positions.";
            return false;
        }
        if (!right_arm_.joint_vel.has_value())
        {
            yError() << "[" + module_name_ + "::encodersMeasUpdate] Error: CubJointControl cannot retrieve RIGHT_ARM joint velocities.";
            return false;
        }
        if (!right_arm_.joint_acc.has_value())
        {
            yError() << "[" + module_name_ + "::encodersMeasUpdate] Error: CubJointControl cannot retrieve RIGHT_ARM joint accelerations.";
            return false;
        }
    }

    if (left_enabled_)
    {
        left_arm_.joint_pos = left_arm_.cjc.getJointValues();
        left_arm_.joint_vel = left_arm_.cjc.getJointSpeeds();
        left_arm_.joint_acc = left_arm_.cjc.getJointAccelerations();

        if (!left_arm_.joint_pos.has_value())
        {
            yError() << "[" + module_name_ + "::encodersMeasUpdate] Error: CubJointControl cannot retrieve LEFT_ARM joint positions.";
            return false;
        }
        if (!left_arm_.joint_vel.has_value())
        {
            yError() << "[" + module_name_ + "::encodersMeasUpdate] Error: CubJointControl cannot retrieve LEFT_ARM joint velocities.";
            return false;
        }
        if (!left_arm_.joint_acc.has_value())
        {
            yError() << "[" + module_name_ + "::encodersMeasUpdate] Error: CubJointControl cannot retrieve LEFT_ARM joint accelerations.";
            return false;
        }
    }

    if (torso_enabled_)
    {
        torso_.joint_pos = torso_.cjc.getJointValues();
        torso_.joint_vel = torso_.cjc.getJointSpeeds();
        torso_.joint_acc = torso_.cjc.getJointAccelerations();

        if (!torso_.joint_pos.has_value())
        {
            yError() << "[" + module_name_ + "::encodersMeasUpdate] Error: CubJointControl cannot retrieve TORSO joint positions.";
            return false;
        }
        if (!torso_.joint_vel.has_value())
        {
            yError() << "[" + module_name_ + "::encodersMeasUpdate] Error: CubJointControl cannot retrieve TORSO joint velocities.";
            return false;
        }
        if (!torso_.joint_acc.has_value())
        {
            yError() << "[" + module_name_ + "::encodersMeasUpdate] Error: CubJointControl cannot retrieve TORSO joint accelerations.";
            return false;
        }
    }

    // Update the vectors of the measured joints of the chains (arms + torso)
    if (right_enabled_)
    {
        if (torso_enabled_)
        {
            right_chain_.measJoints.pos = concatenateEigen(*right_arm_.joint_pos, *torso_.joint_pos);
            right_chain_.measJoints.vel = concatenateEigen(*right_arm_.joint_vel, *torso_.joint_vel);
            right_chain_.measJoints.acc = concatenateEigen(*right_arm_.joint_acc, *torso_.joint_acc);
        }
        else
        {
            right_chain_.measJoints.pos = *right_arm_.joint_pos;
            right_chain_.measJoints.vel = *right_arm_.joint_vel;
            right_chain_.measJoints.acc = *right_arm_.joint_acc;
        }
    }

    if (left_enabled_)
    {
        if (torso_enabled_)
        {
            left_chain_.measJoints.pos = concatenateEigen(*left_arm_.joint_pos, *torso_.joint_pos);
            left_chain_.measJoints.vel = concatenateEigen(*left_arm_.joint_vel, *torso_.joint_vel);
            left_chain_.measJoints.acc = concatenateEigen(*left_arm_.joint_acc, *torso_.joint_acc);
        }
        else
        {
            left_chain_.measJoints.pos = *left_arm_.joint_pos;
            left_chain_.measJoints.vel = *left_arm_.joint_vel;
            left_chain_.measJoints.acc = *left_arm_.joint_acc;
        }
    }

    return true;
}


bool Module::encodersRefUpdate()
{
    // update compound chain joints ref
    compound_chain_.joints.acc = qp_result_.value();
    compound_chain_.joints.vel = compound_chain_.acc2vel_integrator->get_state();
    compound_chain_.joints.pos = compound_chain_.vel2pos_integrator->get_state();

    int nR = right_enabled_ ? right_arm_.cjc.getNumberJoints() : 0;
    int nL = left_enabled_  ? left_arm_.cjc.getNumberJoints()  : 0;
    int nT = torso_enabled_ ? torso_.cjc.getNumberJoints()     : 0;

    int offR = 0;
    int offL = offR + nR;
    int offT = offL + nL;

    if (right_enabled_)
    {
        if (torso_enabled_)
        {
            right_chain_.refJoints.pos = concatenateEigen(compound_chain_.joints.pos.segment(offR, nR), compound_chain_.joints.pos.segment(offT, nT));
            right_chain_.refJoints.vel = concatenateEigen(compound_chain_.joints.vel.segment(offR, nR), compound_chain_.joints.vel.segment(offT, nT));
            right_chain_.refJoints.acc = concatenateEigen(compound_chain_.joints.acc.segment(offR, nR), compound_chain_.joints.acc.segment(offT, nT));
        }
        else
        {
            right_chain_.refJoints.pos = compound_chain_.joints.pos.segment(offR, nR);
            right_chain_.refJoints.vel = compound_chain_.joints.vel.segment(offR, nR);
            right_chain_.refJoints.acc = compound_chain_.joints.acc.segment(offR, nR);
        }
    }

    if (left_enabled_)
    {
        if (torso_enabled_)
        {
            left_chain_.refJoints.pos = concatenateEigen(compound_chain_.joints.pos.segment(offL, nL), compound_chain_.joints.pos.segment(offT, nT));
            left_chain_.refJoints.vel = concatenateEigen(compound_chain_.joints.vel.segment(offL, nL), compound_chain_.joints.vel.segment(offT, nT));
            left_chain_.refJoints.acc = concatenateEigen(compound_chain_.joints.acc.segment(offL, nL), compound_chain_.joints.acc.segment(offT, nT));
        }
        else
        {
            left_chain_.refJoints.pos = compound_chain_.joints.pos.segment(offL, nL);
            left_chain_.refJoints.vel = compound_chain_.joints.vel.segment(offL, nL);
            left_chain_.refJoints.acc = compound_chain_.joints.acc.segment(offL, nL);
        }
    }

    return true;
}


bool Module::measFksUpdate()
{
    //Right
    if (right_enabled_ && right_chain_.measFk)
    {
        right_chain_.measFk->set_joints_state(right_chain_.measJoints.pos, right_chain_.measJoints.vel, right_chain_.measJoints.acc);
        right_chain_.measFk->update();
    }
    if (right_enabled_ && right_chain_.refFk)
    {
        right_chain_.refFk->set_joints_state(right_chain_.refJoints.pos, right_chain_.refJoints.vel, right_chain_.refJoints.acc);
        right_chain_.refFk->update();
    }

    //Left
    if (left_enabled_ && left_chain_.measFk)
    {
        left_chain_.measFk->set_joints_state(left_chain_.measJoints.pos, left_chain_.measJoints.vel, left_chain_.measJoints.acc);
        left_chain_.measFk->update();
    }
    if (left_enabled_ && left_chain_.refFk)
    {
        left_chain_.refFk->set_joints_state(left_chain_.refJoints.pos, left_chain_.refJoints.vel, left_chain_.refJoints.acc);
        left_chain_.refFk->update();
    }

    return true;
}


bool Module::checkAndReadNewInputs()
{

    /*
    * The input can contain zero, one or two end effectors, the dimension of the input vector is:
    * - 7 for each EE for the pose (position + quaternion)
    * - 6 for each EE for the linear and angular velocity
    * The accepted formats are: 7*ee, 13*ee, 19*ee.
    */
    auto setPoseRight = [&](const Eigen::VectorXd& r_pose)
    {
        right_desired_pose_ = Eigen::Translation3d(r_pose.head(3));
        Eigen::Vector4d q = r_pose.tail(4);
        right_desired_pose_.rotate(Eigen::Quaterniond(q));
    };
    auto setPoseLeft = [&](const Eigen::VectorXd& l_pose)
    {
        left_desired_pose_ = Eigen::Translation3d(l_pose.head(3));
        Eigen::Vector4d q = l_pose.tail(4);
        left_desired_pose_.rotate(Eigen::Quaterniond(q));
    };

    auto setVel = [&](bool right, const Eigen::VectorXd& v)
    {
        if (right) { right_desired_lin_vel_ = v.head(3); right_desired_ang_vel_ = v.tail(3); }
        else       { left_desired_lin_vel_  = v.head(3); left_desired_ang_vel_  = v.tail(3); }
    };
    auto setAcc = [&](bool right, const Eigen::VectorXd& a)
    {
        if (right) { right_desired_lin_acc_ = a.head(3); right_desired_ang_acc_ = a.tail(3); }
        else       { left_desired_lin_acc_  = a.head(3); left_desired_ang_acc_  = a.tail(3); }
    };

    auto zeroVel = [&]() {
        right_desired_lin_vel_.setZero(); right_desired_ang_vel_.setZero();
        left_desired_lin_vel_.setZero();  left_desired_ang_vel_.setZero();
    };
    auto zeroAcc = [&]() {
        right_desired_lin_acc_.setZero(); right_desired_ang_acc_.setZero();
        left_desired_lin_acc_.setZero();  left_desired_ang_acc_.setZero();
    };

    yarp::sig::Vector* input = bp_cmd_port_.read(false);
    if (input == nullptr )
        return false;

    int ee = (right_enabled_?1:0) + (left_enabled_?1:0);
    if (ee == 0)
        return false; // nessuna EE da controllare

    auto next = [&](int n, int& idx) {
        Eigen::VectorXd out = yarp::eigen::toEigen(input->subVector(idx, idx + n - 1));
        idx += n;
        return out;
    };

    int idx = 0;
    const int nPose = 7*ee;
    const int nVel  = 6*ee;
    const int nAcc  = 6*ee;

    if (input->size() == nPose || input->size() == nPose + nVel || input->size() == nPose + nVel + nAcc)
    {
        // POSE
        if (right_enabled_) { setPoseRight(next(7, idx)); }
        if (left_enabled_)  { setPoseLeft(next(7, idx)); }

        // VEL
        if ((int)input->size() >= nPose + nVel)
        {
            if (right_enabled_) { setVel(true,  next(6, idx)); }
            if (left_enabled_)  { setVel(false, next(6, idx)); }
        }
        else
        {
            zeroVel();
        }

        // ACC
        if ((int)input->size() == nPose + nVel + nAcc)
        {
            if (right_enabled_) { setAcc(true,  next(6, idx)); }
            if (left_enabled_)  { setAcc(false, next(6, idx)); }
        }
        else
        {
            zeroAcc();
        }

        yInfo()<< "[" + module_name_ + "::checkAndReadNewInputs] Received new desired trajectory data.";
        return true;
    }
    else
    {
        // The received format is not correct: remain in the actual pose and set the velocities and the accelerations to zero
        zeroVel();
        zeroAcc();
        yInfo()<< "[" + module_name_ + "::checkAndReadNewInputs] Received wrong inputs. Keeping last desired poses.";
        return true;
    }
}


bool Module::refFksUpdate()
{
    if (right_enabled_ && right_chain_.refFk)
    {
        right_chain_.refFk->set_joints_state(right_chain_.refJoints.pos, right_chain_.refJoints.vel, right_chain_.refJoints.acc);
        right_chain_.refFk->update();
    }

    if (left_enabled_ && left_chain_.refFk)
    {
        left_chain_.refFk->set_joints_state(left_chain_.refJoints.pos, left_chain_.refJoints.vel, left_chain_.refJoints.acc);
        left_chain_.refFk->update();
    }

    return true;
}


void Module::ikUpdate()
{
    Eigen::Affine3d I = Eigen::Affine3d::Identity();
    Eigen::Vector3d z3 = Eigen::Vector3d::Zero();
    Eigen::MatrixXd J0(6, 0); // 6x0 per "nessuna" catena
    J0.setZero();
    Eigen::VectorXd b0 = Eigen::VectorXd::Zero(6);

    // RIGHT (values, non reference)
    Eigen::Affine3d  rT  = (right_enabled_ && right_chain_.refFk) ? right_chain_.refFk->get_ee_transform() : I;
    Eigen::Vector3d  rVl = (right_enabled_ && right_chain_.refFk) ? right_chain_.refFk->get_ee_lin_vel()   : z3;
    Eigen::Vector3d  rVa = (right_enabled_ && right_chain_.refFk) ? right_chain_.refFk->get_ee_ang_vel()   : z3;
    Eigen::Vector3d  rAl = (right_enabled_ && right_chain_.refFk) ? right_chain_.refFk->get_ee_lin_acc()   : z3;
    Eigen::Vector3d  rAa = (right_enabled_ && right_chain_.refFk) ? right_chain_.refFk->get_ee_ang_acc()   : z3;
    Eigen::MatrixXd  rJ  = (right_enabled_ && right_chain_.refFk) ? right_chain_.refFk->get_jacobian()     : J0;
    Eigen::VectorXd  rB  = (right_enabled_ && right_chain_.refFk) ? right_chain_.refFk->get_ee_bias_acc()  : b0;

    // LEFT (values, non reference)
    Eigen::Affine3d  lT  = (left_enabled_ && left_chain_.refFk) ? left_chain_.refFk->get_ee_transform() : I;
    Eigen::Vector3d  lVl = (left_enabled_ && left_chain_.refFk) ? left_chain_.refFk->get_ee_lin_vel()   : z3;
    Eigen::Vector3d  lVa = (left_enabled_ && left_chain_.refFk) ? left_chain_.refFk->get_ee_ang_vel()   : z3;
    Eigen::Vector3d  lAl = (left_enabled_ && left_chain_.refFk) ? left_chain_.refFk->get_ee_lin_acc()   : z3;
    Eigen::Vector3d  lAa = (left_enabled_ && left_chain_.refFk) ? left_chain_.refFk->get_ee_ang_acc()   : z3;
    Eigen::MatrixXd  lJ  = (left_enabled_ && left_chain_.refFk) ? left_chain_.refFk->get_jacobian()     : J0;
    Eigen::VectorXd  lB  = (left_enabled_ && left_chain_.refFk) ? left_chain_.refFk->get_ee_bias_acc()  : b0;

    compound_chain_.ik->set_robot_state(
        compound_chain_.vel2pos_integrator->get_state(),
        compound_chain_.acc2vel_integrator->get_state(),
        rT, rVl, rVa, rAl, rAa, rJ, rB,
        lT, lVl, lVa, lAl, lAa, lJ, lB
    );
}



void Module::setDesiredTrajectory()
{
    Eigen::Affine3d rPose = right_enabled_ ? right_desired_pose_ : Eigen::Affine3d::Identity();
    Eigen::Affine3d lPose = left_enabled_  ? left_desired_pose_  : Eigen::Affine3d::Identity();

    Eigen::Vector3d rLv   = right_enabled_ ? right_desired_lin_vel_ : Eigen::Vector3d::Zero();
    Eigen::Vector3d rAv   = right_enabled_ ? right_desired_ang_vel_ : Eigen::Vector3d::Zero();
    Eigen::Vector3d lLv   = left_enabled_  ? left_desired_lin_vel_  : Eigen::Vector3d::Zero();
    Eigen::Vector3d lAv   = left_enabled_  ? left_desired_ang_vel_  : Eigen::Vector3d::Zero();

    Eigen::Vector3d rLa   = right_enabled_ ? right_desired_lin_acc_ : Eigen::Vector3d::Zero();
    Eigen::Vector3d rAa   = right_enabled_ ? right_desired_ang_acc_ : Eigen::Vector3d::Zero();
    Eigen::Vector3d lLa   = left_enabled_  ? left_desired_lin_acc_  : Eigen::Vector3d::Zero();
    Eigen::Vector3d lAa   = left_enabled_  ? left_desired_ang_acc_  : Eigen::Vector3d::Zero();

    compound_chain_.ik->set_desired_ee_transform(rPose, lPose);
    compound_chain_.ik->set_desired_ee_twist(rLv, rAv, lLv, lAv);
    compound_chain_.ik->set_desired_ee_acceleration(rLa, rAa, lLa, lAa);
}


bool Module::solveIkAndUpdateIntegrators()
{
    qp_result_ = compound_chain_.ik->solve();

    if (!qp_result_.has_value())
    {
        yError()<< "[" + module_name_ + "::solveIkAndUpdateIntegrators] Error: No value for the ik solution!";
        return false;
    }

    // Update integrators
    compound_chain_.acc2vel_integrator->integrate(qp_result_.value());
    compound_chain_.vel2pos_integrator->integrate(compound_chain_.acc2vel_integrator->get_state());

    return true;
}


bool Module::moveChains()
{
    Eigen::VectorXd joint_refs = compound_chain_.joints.pos;

    int nR = right_enabled_ ? right_arm_.cjc.getNumberJoints() : 0;
    int nL = left_enabled_  ? left_arm_.cjc.getNumberJoints()  : 0;
    int nT = torso_enabled_ ? torso_.cjc.getNumberJoints()     : 0;

    int offset = 0;

    if (right_enabled_)
    {
        if(!right_arm_.cjc.moveToStreaming(joint_refs.segment(offset, nR)))
        {
            yError() << "[" + module_name_ + "::moveChains] Error: Cannot move RIGHT_ARM. See the errors above.";
            return false;
        }
        offset += nR;
    }
    if (left_enabled_)
    {
        if(!left_arm_.cjc.moveToStreaming(joint_refs.segment(offset, nL)))
        {
            yError() << "[" + module_name_ + "::moveChains] Error: Cannot move LEFT_ARM. See the errors above.";
            return false;
        }
        offset += nL;
    }
    if (torso_enabled_)
    {
        if(!torso_.cjc.moveToStreaming(joint_refs.segment(offset, nT)))
        {
            yError() << "[" + module_name_ + "::moveChains] Error: Cannot move TORSO. See the errors above.";
            return false;
        }
        offset += nT;
    }

    return true;
}


bool Module::isMotionDone()
{
    auto ik_joint_vel_abs = compound_chain_.joints.vel.array().abs();

    for (int i = 0; i < ik_joint_vel_abs.size(); ++i)
    {
        if (ik_joint_vel_abs(i) > compound_chain_.stop_vel)
        {
            return false;
        }
    }

    return true;
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


void Module::verboseAndLog()
{
    struct verboseAndLogValues
    {
        Eigen::VectorXd jnt_pos, jnt_vel, jnt_acc;
        Eigen::Matrix3d cart_rot;
        Eigen::AngleAxisd cart_rot_aa;
        Eigen::Quaterniond cart_rot_quat;
        Eigen::AngleAxisd cart_rot_aa_err;
        Eigen::Vector3d cart_pos, cart_pos_err;
        Eigen::VectorXd ee_bias_acc;
        double manip, manip_func;
    };

    struct verboseAndLogStruct
    {
        verboseAndLogValues des, qp, meas;
    } right, left;


    static double right_max_manip = 0.0, left_max_manip = 0.0;

    auto fillVerboseAndLogStruct = [] (verboseAndLogStruct& vals, double& max_manip, const Eigen::Affine3d& des_pose, const Module::CHAINS& chain)
    {
        //Desired values
        vals.des.cart_pos = des_pose.translation();
        vals.des.cart_rot = des_pose.rotation();
        vals.des.cart_rot_aa = Eigen::AngleAxisd(vals.des.cart_rot);
        vals.des.cart_rot_quat = Eigen::Quaterniond(vals.des.cart_rot_aa);

        //Reference values
        vals.qp.jnt_pos = chain.refJoints.pos;
        vals.qp.jnt_vel = chain.refJoints.vel;
        vals.qp.jnt_acc = chain.refJoints.acc;
        vals.qp.cart_pos = chain.refFk->get_ee_transform().translation();
        vals.qp.cart_rot = chain.refFk->get_ee_transform().rotation();
        vals.qp.cart_rot_aa = Eigen::AngleAxisd(vals.qp.cart_rot);
        vals.qp.cart_rot_quat = Eigen::Quaterniond(vals.qp.cart_rot_aa);
        vals.qp.ee_bias_acc = chain.refFk->get_ee_bias_acc();

        vals.qp.cart_pos_err = vals.des.cart_pos - vals.qp.cart_pos;
        vals.qp.cart_rot_aa_err = Eigen::AngleAxisd(vals.des.cart_rot * vals.qp.cart_rot.transpose());

        auto qp_jacobian = chain.refFk->get_jacobian();
        vals.qp.manip = sqrt((qp_jacobian * qp_jacobian.transpose()).determinant());
        max_manip = std::max(vals.qp.manip,max_manip);
        vals.qp.manip_func =  pow(1 - (vals.qp.manip / max_manip), 2);

        //Measured values
        vals.meas.jnt_pos = chain.measJoints.pos;
        vals.meas.jnt_vel = chain.measJoints.vel;
        vals.meas.jnt_acc = chain.measJoints.acc;
        vals.meas.cart_pos = chain.measFk->get_ee_transform().translation();
        vals.meas.cart_rot = chain.measFk->get_ee_transform().rotation();
        vals.meas.cart_rot_aa = Eigen::AngleAxisd(vals.meas.cart_rot);
        vals.meas.cart_rot_quat = Eigen::Quaterniond(vals.meas.cart_rot_aa);
        vals.meas.ee_bias_acc = chain.measFk->get_ee_bias_acc();

        vals.meas.cart_pos_err = vals.qp.cart_pos - vals.meas.cart_pos;
        vals.meas.cart_rot_aa_err = Eigen::AngleAxisd(vals.qp.cart_rot * vals.meas.cart_rot.transpose());
    };

    /* RIGHT*/
    if (right_enabled_ && right_chain_.refFk && right_chain_.measFk)
        fillVerboseAndLogStruct(right, right_max_manip, right_desired_pose_, right_chain_);

    /* LEFT*/
    if (left_enabled_ && left_chain_.refFk && left_chain_.measFk)
        fillVerboseAndLogStruct(left, left_max_manip, left_desired_pose_, left_chain_);

    if (module_verbose_)
    {
        auto printVerboseAndLogStruct = [] (const verboseAndLogStruct& vals, const double max_manip)
        {
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
            yInfo() << "---------- Desired input --------------------------";
            yInfo() << "pos des" << eigenToString(vals.des.cart_pos);
            yInfo() << "ori des" << eigenToString(vals.des.cart_rot_aa.axis() * vals.des.cart_rot_aa.angle());
            yInfo() << "ori qua: "<<vals.des.cart_rot_quat.x()<<" "<<vals.des.cart_rot_quat.y()<<" "<<vals.des.cart_rot_quat.z()<<" "<<vals.des.cart_rot_quat.w();

            //QP generated values
            yInfo() << "---------- QP generated values, errors w.r.t. input ----------";
            yInfo() << "pos gen" << eigenToString(vals.qp.cart_pos);
            yInfo() << "ori gen" << eigenToString(vals.qp.cart_rot_aa.axis() * vals.qp.cart_rot_aa.angle());
            yInfo() <<" ori qua: "<<vals.qp.cart_rot_quat.x()<<" "<<vals.qp.cart_rot_quat.y()<<" "<<vals.qp.cart_rot_quat.z()<<" "<<vals.qp.cart_rot_quat.w();
            yInfo() << "pos err (m) |norm| [components]" << vals.qp.cart_pos_err.norm() << "\t" << eigenToString(vals.qp.cart_pos_err);
            yInfo() << "ang err (deg) |norm| [components]" << vals.qp.cart_rot_aa_err.angle() * (180 / M_PI) << "\t" << eigenToString(vals.qp.cart_rot_aa_err.angle() * vals.qp.cart_rot_aa_err.axis());
            yInfo() << "joints qp acc" << eigenToString(vals.qp.jnt_acc);
            yInfo() << "joints qp vel" << eigenToString(vals.qp.jnt_vel);
            yInfo() << "joints qp pos" << eigenToString(vals.qp.jnt_pos);
            yInfo() << "manip "<<vals.qp.manip<<" max_manip "<<max_manip<<" manip/max_manip "<<(max_manip>0?vals.qp.manip/max_manip:0.0)<<" weight_manip_function "<<vals.qp.manip_func;

            //measured values
            yInfo() << "---------- Current values, errors w.r.t. QP generated -----------";
            yInfo() << "pos cur" << eigenToString(vals.meas.cart_pos);
            yInfo() << "ori cur" << eigenToString(vals.meas.cart_rot_aa.axis() * vals.meas.cart_rot_aa.angle())<<"\tquat: "<<eigenToString(vals.meas.cart_rot_quat.coeffs());
            yInfo() << "pos err (m) |norm| [components]" << vals.meas.cart_pos_err.norm() << "\t" << eigenToString(vals.meas.cart_pos_err);
            yInfo() << "ang err (deg) |norm| [components]" << vals.meas.cart_rot_aa_err.angle() * (180 / M_PI) << "\t" << eigenToString(vals.meas.cart_rot_aa_err.angle() * vals.meas.cart_rot_aa_err.axis());
            yInfo() << "joints meas acc" << eigenToString(vals.meas.jnt_acc);
            yInfo() << "joints meas vel" << eigenToString(vals.meas.jnt_vel);
            yInfo() << "joints meas pos" << eigenToString(vals.meas.jnt_pos);
            yInfo() << "################################################################";
        };

        yInfo() << "[" + module_name_ + "::verboseAndLog] Verbose ******************************";


        yInfo() << "---------- STATE -----------";
        auto state = getState();
        if (state == State::Stop)
            {yInfo() << "state\t|Stop|";}
        else if (state == State::Running)
            yInfo() << "state\t|Running|";

        if (right_enabled_ && right_chain_.refFk && right_chain_.measFk)
        {
            yInfo() << "---------- RIGHT CHAIN -----------";
            printVerboseAndLogStruct(right, right_max_manip);
        }

        if (left_enabled_ && left_chain_.refFk && left_chain_.measFk)
        {
            yInfo() << "---------- LEFT CHAIN -----------";
            printVerboseAndLogStruct(left, left_max_manip);
        }
    }

    if(module_logging_)
    {
        yInfo() << "[" + module_name_ + "::verboseAndLog] Logging.";
    }

}


void Module::appendEigen(Eigen::VectorXd &vec, const Eigen::VectorXd &vec_app)
{
    Eigen::VectorXd temp(vec);
    vec.resize(temp.size() + vec_app.size());
    if (temp.size() > 0) vec << temp, vec_app; else vec << vec_app;
};

Eigen::VectorXd Module::concatenateEigen(const Eigen::VectorXd &vec1, const Eigen::VectorXd &vec2)
{
    Eigen::VectorXd vec(vec1.size()+vec2.size());
    vec << vec1,vec2;
    return vec;
};