#include <ergocub_cartesian_bimanual/module.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>
#include <yarp/eigen/Eigen.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <utils/utils.h>
#include <utils/utils.hpp>

#include <optional>

#include <array>
#include <chrono>
#include <thread>
#include <vector>

#include <unsupported/Eigen/MatrixFunctions>

using namespace std::literals::chrono_literals;

Module::Module(const std::string &module_name):module_name_(module_name){}

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
        ||  !(utils::checkParameters({{"qp_verbose"}}, "", COMMON_bot, "", utils::ParameterType::Bool, false))
        ||  !(utils::checkParameters({{"query_port_name", "input_port_name"}}, "", COMMON_bot, "", utils::ParameterType::String, false)))
    {
        yError() << "[" + module_name_ + "::configure] Error: mandatory parameter(s) for COMMON group missing or invalid.";
        return false;
    }

    sample_time_ = 1.0 / COMMON_bot.find("rate").asFloat64();
    const bool qp_verbose = COMMON_bot.find("qp_verbose").asBool();
    query_port_.open("/" + module_name_ + COMMON_bot.find("query_port_name").asString());
    input_cmd_.open("/" + module_name_ + COMMON_bot.find("input_port_name").asString());
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

        if  (   !(utils::checkParameters({{"improve_manip_dyn", "improve_manip_th", "joint_limits_param", "cartesian_pos_weight", "cartesian_pos_p_gain", "cartesian_pos_d_gain", "cartesian_ori_weight", "cartesian_ori_p_gain", "cartesian_ori_d_gain", "stop_vel"}}, "", RIGHT_ARM_bot, "", utils::ParameterType::Float64, false))
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

        if  (   !(utils::checkParameters({{"improve_manip_dyn", "improve_manip_th", "joint_limits_param", "cartesian_pos_weight", "cartesian_pos_p_gain", "cartesian_pos_d_gain", "cartesian_ori_weight", "cartesian_ori_p_gain", "cartesian_ori_d_gain", "stop_vel"}}, "", LEFT_ARM_bot, "", utils::ParameterType::Float64, false))
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

        if  (   !(utils::checkParameters({{"joint_limits_param"}}, "", TORSO_bot, "", utils::ParameterType::Float64, false))
            ||  !(utils::checkParameters({{"joint_pos_weight", "joint_pos_p_gain", "joint_pos_d_gain"}}, "", TORSO_bot, "", utils::ParameterType::Float64, true)))
        {
            yError() << "[" + module_name_ + "::configure] Error: mandatory parameter(s) for TORSO group missing or invalid.";
            return false;
        }
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
        Eigen::VectorXd joint_limits_params;
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

            appendOrInit(joint_pos_weights, utils::loadVectorDouble(RIGHT_ARM_bot, "joint_pos_weight"));
            appendOrInit(joint_pos_p_gain,  utils::loadVectorDouble(RIGHT_ARM_bot, "joint_pos_p_gain"));
            appendOrInit(joint_pos_d_gain,  utils::loadVectorDouble(RIGHT_ARM_bot, "joint_pos_d_gain"));

            appendOrInit(joint_home, *right_arm_.joint_pos);
        }
        // LEFT ARM (joint e cartesian)
        if (left_enabled_)
        {
            appendOrInit(joint_limits_params, Eigen::Vector<double,1>(LEFT_ARM_bot.find("joint_limits_param").asFloat64()));

            appendOrInit(joint_pos_weights, utils::loadVectorDouble(LEFT_ARM_bot, "joint_pos_weight"));
            appendOrInit(joint_pos_p_gain,  utils::loadVectorDouble(LEFT_ARM_bot, "joint_pos_p_gain"));
            appendOrInit(joint_pos_d_gain,  utils::loadVectorDouble(LEFT_ARM_bot, "joint_pos_d_gain"));

            appendOrInit(joint_home, *left_arm_.joint_pos);
        }
        // TORSO (only joint terms)
        if (torso_enabled_)
        {
            appendOrInit(joint_limits_params, Eigen::Vector<double,1>(TORSO_bot.find("joint_limits_param").asFloat64()));

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

        // RIGHT AND LEFT ARM: stop_vel
        std::vector<double> stop_vels, manip_dyns, manip_ths;
        if (right_enabled_) { stop_vels.push_back(RIGHT_ARM_bot.find("stop_vel").asFloat64());
                              manip_dyns.push_back(RIGHT_ARM_bot.find("improve_manip_dyn").asFloat64());
                              manip_ths.push_back(RIGHT_ARM_bot.find("improve_manip_th").asFloat64()); }
        if (left_enabled_)  { stop_vels.push_back(LEFT_ARM_bot.find("stop_vel").asFloat64());
                              manip_dyns.push_back(LEFT_ARM_bot.find("improve_manip_dyn").asFloat64());
                              manip_ths.push_back(LEFT_ARM_bot.find("improve_manip_th").asFloat64()); }

        compound_chain_.stop_vel = stop_vels.empty() ? 1e-3 : *std::min_element(stop_vels.begin(), stop_vels.end());
        double improve_manip_dyn = manip_dyns.empty() ? 0.0 : *std::min_element(manip_dyns.begin(), manip_dyns.end());
        double improve_manip_th = manip_ths.empty() ? 0.0 : *std::min_element(manip_ths.begin(), manip_ths.end());

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
                                                                                joint_pos_weights,
                                                                                joint_pos_p_gain,
                                                                                joint_pos_d_gain,
                                                                                cartesian_pos_weight,
                                                                                cartesian_pos_p_gain,
                                                                                cartesian_pos_d_gain,
                                                                                cartesian_ori_weight,
                                                                                cartesian_ori_p_gain,
                                                                                cartesian_ori_d_gain,
                                                                                improve_manip_dyn,
                                                                                improve_manip_th,
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

    #ifdef LOGGING_RERUN
    logger_ = std::make_unique<rerun::RecordingStream>(module_name_);
    logger_->spawn().handle();

    auto jointSeriesNames = [] (const std::vector<std::string>& jointNames)
    {
        std::vector<rerun::components::Name> names;
        names.reserve(2 * jointNames.size());

        for (const auto& jointName : jointNames)
            names.emplace_back("ref/" + jointName);
        for (const auto& jointName : jointNames)
            names.emplace_back("meas/" + jointName);

        return names;
    };

    auto logJointSeriesNames = [&] (const CHAINS& chain, const std::string& log_string)
    {
        if (!chain.refFk)
            return;

        const auto jointNames = chain.refFk->get_joints_list();
        logger_->log_static(
            "/" + module_name_ + "/" + log_string + "/joints",
            rerun::SeriesLines().with_names(jointSeriesNames(jointNames))
        );
    };

    logger_->set_time_duration_secs(timeLine_, yarp::os::Time::now());
    if(right_enabled_)  logJointSeriesNames(right_chain_, "right");
    if(left_enabled_)   logJointSeriesNames(left_chain_, "left");
    #endif

    setState(State::Stop);

    yInfo() << "[" + module_name_ + "::" + __func__ + "] Done.";

    return true;
}

bool Module::close()
{
    query_port_.close();
    input_cmd_.close();
    joints_pos_port_.close();

    return true;
}

double Module::getPeriod()
{
    return sample_time_;
}

bool Module::interruptModule()
{
    query_port_.interrupt();
    input_cmd_.interrupt();
    joints_pos_port_.interrupt();

    return true;
}

bool Module::updateModule()
{

    static const int MAX_ATTEMPT = 5;
    static int attempt = 0;
    if(!encodersMeasUpdate()){
        attempt++;
        if (attempt >= MAX_ATTEMPT){
            yError() << "[" + module_name_ + "::" + __func__ + "] Cannot read encoders after " << MAX_ATTEMPT << " attempts. See error(s) above.";
            return false;
        }
        yWarning() << "[" + module_name_ + "::" + __func__ + "] Cannot read encoders. Attempt " << attempt << " of " << MAX_ATTEMPT << ". See error(s) above.";
    }
    else
    {
        attempt = 0; // reset attempt counter if encoders read successfully
    }

    measFksUpdate();

    if(checkAndReadNewInputs())
    {
        setState(State::Running);
    }

    if(getState()==State::Running)
    {
        refFksUpdate();
        
        ikUpdate();
        setDesiredTrajectory();

        if(!solveIkAndUpdateIntegrators())
        {
            yError()<< "[" + module_name_ + "::" + __func__ + "] See error(s) above.";
            return false;
        }
        
        if (!encodersRefUpdate())
        {
            yError()<< "[" + module_name_ + "::" + __func__ + "] See error(s) above.";
            return false;
        }
        if (!no_control_)
        {
            if(!moveChains())
            {
                yError()<< "[" + module_name_ + "::" + __func__ + "] See error(s) above.";
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

    checkAndReadQuery();

    #ifdef LOGGING_RERUN
    log();
    #endif

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
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: CubJointControl cannot retrieve RIGHT_ARM joint positions.";
            return false;
        }
        if (!right_arm_.joint_vel.has_value())
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: CubJointControl cannot retrieve RIGHT_ARM joint velocities.";
            return false;
        }
        if (!right_arm_.joint_acc.has_value())
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: CubJointControl cannot retrieve RIGHT_ARM joint accelerations.";
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
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: CubJointControl cannot retrieve LEFT_ARM joint positions.";
            return false;
        }
        if (!left_arm_.joint_vel.has_value())
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: CubJointControl cannot retrieve LEFT_ARM joint velocities.";
            return false;
        }
        if (!left_arm_.joint_acc.has_value())
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: CubJointControl cannot retrieve LEFT_ARM joint accelerations.";
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
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: CubJointControl cannot retrieve TORSO joint positions.";
            return false;
        }
        if (!torso_.joint_vel.has_value())
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: CubJointControl cannot retrieve TORSO joint velocities.";
            return false;
        }
        if (!torso_.joint_acc.has_value())
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: CubJointControl cannot retrieve TORSO joint accelerations.";
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
    * - 12 for each EE for the pose (position + matrix) or 7 for each EE for the pose (position + quaternion)
    * - 6 for each EE for the linear and angular velocity
    * - 6 for each EE for the linear and angular acceleration
    * The accepted formats are: 12*ee, 18*ee, 24*ee or 7*ee, 13*ee, 19*ee,
    * where ee is the number of end effectors (1 or 2) contained in the input vector.
    */
    auto setPoseMat = [&](bool right, const Eigen::VectorXd& pose)
    {
        if (right) {
            right_desired_pose_ = Eigen::Translation3d(pose.head(3));
            right_desired_pose_.rotate(Eigen::Matrix3d(pose.tail(9).reshaped(3,3)));
        }
        else {
            left_desired_pose_ = Eigen::Translation3d(pose.head(3));
            left_desired_pose_.rotate(Eigen::Matrix3d(pose.tail(9).reshaped(3,3)));
        }
    };

    auto setPoseQuat = [&](bool right, const Eigen::VectorXd& pose)
    {
        if (right) {
            right_desired_pose_ = Eigen::Translation3d(pose.head(3));
            right_desired_pose_.rotate(Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]));
        }
        else {
            left_desired_pose_ = Eigen::Translation3d(pose.head(3));
            left_desired_pose_.rotate(Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]));
        }
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

    yarp::sig::Vector* input = input_cmd_.read(false);
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
    const int nPose_mat = 12*ee;
    const int nPose_quat = 7*ee;
    const int nVel  = 6*ee;
    const int nAcc  = 6*ee;

    if (input->size() == nPose_mat || input->size() == nPose_mat + nVel || input->size() == nPose_mat + nVel + nAcc)
    {
        // POSE
        if (right_enabled_) { setPoseMat(true, next(12, idx)); }
        if (left_enabled_)  { setPoseMat(false, next(12, idx)); }

        // VEL
        if ((int)input->size() >= nPose_mat + nVel)
        {
            if (right_enabled_) { setVel(true,  next(6, idx)); }
            if (left_enabled_)  { setVel(false, next(6, idx)); }
        }
        else
        {
            zeroVel();
        }

        // ACC
        if ((int)input->size() == nPose_mat + nVel + nAcc)
        {
            if (right_enabled_) { setAcc(true,  next(6, idx)); }
            if (left_enabled_)  { setAcc(false, next(6, idx)); }
        }
        else
        {
            zeroAcc();
        }

        return true;
    }
    else if (input->size() == nPose_quat || input->size() == nPose_quat + nVel || input->size() == nPose_quat + nVel + nAcc)
    {
        // POSE
        if (right_enabled_) { setPoseQuat(true, next(7, idx)); }
        if (left_enabled_)  { setPoseQuat(false, next(7, idx)); }

        // VEL
        if ((int)input->size() >= nPose_quat + nVel)
        {
            if (right_enabled_) { setVel(true,  next(6, idx)); }
            if (left_enabled_)  { setVel(false, next(6, idx)); }
        }
        else
        {
            zeroVel();
        }

        // ACC
        if ((int)input->size() == nPose_quat + nVel + nAcc)
        {
            if (right_enabled_) { setAcc(true,  next(6, idx)); }
            if (left_enabled_)  { setAcc(false, next(6, idx)); }
        }
        else
        {
            zeroAcc();
        }

        return true;
    }
    else
    {
        // The received format is not correct: remain in the actual pose and set the velocities and the accelerations to zero
        zeroVel();
        zeroAcc();
        yInfo()<< "[" + module_name_ + "::" + __func__ + "] Received wrong inputs. Keeping last desired poses.";
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
        yError()<< "[" + module_name_ + "::" + __func__ + "] Error: No value for the ik solution!";
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
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: Cannot move RIGHT_ARM. See the errors above.";
            return false;
        }
        offset += nR;
    }
    if (left_enabled_)
    {
        if(!left_arm_.cjc.moveToStreaming(joint_refs.segment(offset, nL)))
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: Cannot move LEFT_ARM. See the errors above.";
            return false;
        }
        offset += nL;
    }
    if (torso_enabled_)
    {
        if(!torso_.cjc.moveToStreaming(joint_refs.segment(offset, nT)))
        {
            yError() << "[" + module_name_ + "::" + __func__ + "] Error: Cannot move TORSO. See the errors above.";
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

#ifdef LOGGING_RERUN
void Module::log()
{
    if (!logger_)
        return;

    auto logTransform3D = [&] (const std::string& entity, const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation)
    {
        const std::array<float, 9> rotationColumns{
            static_cast<float>(rotation(0, 0)),
            static_cast<float>(rotation(1, 0)),
            static_cast<float>(rotation(2, 0)),
            static_cast<float>(rotation(0, 1)),
            static_cast<float>(rotation(1, 1)),
            static_cast<float>(rotation(2, 1)),
            static_cast<float>(rotation(0, 2)),
            static_cast<float>(rotation(1, 2)),
            static_cast<float>(rotation(2, 2)),
        };

        logger_->log(entity,
            rerun::Transform3D(
                rerun::components::Translation3D(position.x(), position.y(), position.z()),
                rerun::components::TransformMat3x3(rerun::datatypes::Mat3x3(rotationColumns))
            ), rerun::archetypes::TransformAxes3D(0.1f)
        );
    };

    auto jointValues = [] (const Eigen::VectorXd& refJoints, const Eigen::VectorXd& measJoints)
    {
        std::vector<double> values;
        values.reserve(static_cast<std::size_t>(refJoints.size() + measJoints.size()));
        values.insert(values.end(), refJoints.data(), refJoints.data() + refJoints.size());
        values.insert(values.end(), measJoints.data(), measJoints.data() + measJoints.size());
        return values;
    };

    logger_->set_time_duration_secs(timeLine_, yarp::os::Time::now());

    auto logChain = [&] (const CHAINS& chain, const std::string& log_string){
        if (!chain.refFk || !chain.measFk)
            return;

        logTransform3D("/" + module_name_ + "/" + log_string + "/ref/pose", chain.refFk->get_ee_transform().translation(), chain.refFk->get_ee_transform().rotation());
        logTransform3D("/" + module_name_ + "/" + log_string + "/meas/pose", chain.measFk->get_ee_transform().translation(), chain.measFk->get_ee_transform().rotation());

        logger_->log(
            "/" + module_name_ + "/" + log_string + "/joints",
            rerun::Scalars(jointValues(chain.refJoints.pos, chain.measJoints.pos))
        );
    };

    logger_->set_time_duration_secs(timeLine_, yarp::os::Time::now());

    if(right_enabled_)  logChain(right_chain_, "right");
    if(left_enabled_)   logChain(left_chain_, "left");

}
#endif

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
