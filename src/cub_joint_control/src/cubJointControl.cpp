// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include <cub-joint-control/cubJointControl.h>
#include <utils/utils.hpp>

#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/dev/IPositionControl.h>

CubJointControl::~CubJointControl()
{
    yDebug() << class_name_ + "::~CubJointControl(). Closing the CubJointControl for the end-effector with name '" + ee_name_ + "'.";
    /* Restore the original control mode. */
    if (control_mode_ != nullptr)
    {
        if (!control_mode_->setControlModes(joints_modes_original_.data()))
        {
            yError() << class_name_ << "::~CubJointControl(). Error: cannot restore original joints control modes for the end-effector with name '" + ee_name_ + "'.";
        }
    }

    /* Restore the original joints speeds. */
    if (p_control_ != nullptr)
    {
        if (!p_control_->setRefSpeeds(joints_speeds_original_.data()))
        {
            yError() << class_name_ << "::~CubJointControl(). Error: cannot restore original joints speeds for the end-effector with name '" + ee_name_ + "'.";
        }
    }

    yDebug() << class_name_ + "::~CubJointControl(). Closing the PolyDriver for the end-effector with name '" + ee_name_ + "'.";

    /* Close the driver. */
    if (drv_.isValid())
    {
        if (!drv_.close())
        {
            yError() << class_name_ << "::~CubJointControl(). Error: cannot close the driver for the end-effector with name '" + ee_name_ + "'.";
        }
    }

    yDebug() << class_name_ + "::~CubJointControl(). CubJointControl for the end-effector with name '" + ee_name_ + "' closed successfully.";
}


std::size_t CubJointControl::getNumberJoints()
{
    return joints_.size();
}


bool CubJointControl::setMode(const std::string& mode)
{
    yarp::conf::vocab32_t des_mode;

    if (mode == "NonStreaming")
    {
        des_mode = VOCAB_CM_POSITION;
    }
    else if (mode == "Streaming")
    {
        des_mode = VOCAB_CM_POSITION_DIRECT;
    }
    else
    {
        des_mode = VOCAB_CM_UNKNOWN;
    }

    std::vector<int> modes(joints_.size(), des_mode);
    if (!control_mode_->setControlModes(modes.data()))
    {
        yError() << class_name_ + "::setMode(). Error: Cannot set desired control mode for the joints of the end-effector with name '" + ee_name_ + "' using IControlMode::setControlModes().";
        return false;
    }

    joints_modes_current_ = des_mode;

    return true;
}


bool CubJointControl::setSpeeds(const Eigen::VectorXd& speeds)
{
    if (!p_control_)
    {
        yError() << class_name_ + "::setSpeeds(). Error: p_control_ not defined for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    if (speeds.size() != joints_.size())
    {
        yError() << class_name_ + "::setSpeeds(). Error: speeds.size() should be equal to " + std::to_string(joints_.size()) + "for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    /* Convert to degrees per seconds. */
    Eigen::VectorXd speeds_deg = speeds * 180.0 / M_PI;

    if (!p_control_->setRefSpeeds(speeds_deg.data()))
    {
        yError() << class_name_ + "::setSpeeds(). Error: cannot call IPositionControl::setRefSpeeds() for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    return true;
}


bool CubJointControl::stop()
{
    if(joints_modes_current_ == VOCAB_CM_POSITION)
    {
        return p_control_->stop();
    }
    else if (joints_modes_current_ == VOCAB_CM_POSITION_DIRECT)
    {
        /* To stop the controller in this control mode it is sufficient not to send any position reference. */
        return true;
    }

    return false;
}


bool CubJointControl::moveTo(const Eigen::VectorXd& joints, const bool& blocking)
{
    if (!p_control_)
    {
        throw(std::runtime_error(class_name_ + "::isMotionDone(). Error: pointer to yarp::dev::IPositionControl interface is unexpectedly empty. Aborting operations."));
    }

    if (joints.size() != joints_.size())
    {
        yError() << class_name_ + "::moveTo(). Error: joints.size() should be equal to " + std::to_string(joints_.size()) + " for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    /* Convert to degrees. */
    Eigen::VectorXd joints_deg = joints * 180.0 / M_PI;

    /* Let's move. */
    if (!p_control_->positionMove(joints_deg.data()))
    {
        yError() << class_name_ + "::moveTo(). Error: p_control_ cannot set new reference point for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    /* If it is blocking keep checking that motion is done. */
    if (blocking)
    {
        std::optional<bool> is_motion_done = false;
        while (!(*is_motion_done))
        {
            yarp::os::Time::delay(0.1);

            is_motion_done = isMotionDone();
            if (!is_motion_done.has_value())
            {
                yError() << class_name_ + "::moveTo(). Error: cannot check if motion is done.";
                return false;
            }
        }

        return true;
    }

    return true;
}


std::optional<bool> CubJointControl::isMotionDone()
{
    if (!p_control_)
    {
        throw(std::runtime_error(class_name_ + "::isMotionDone(). Error: pointer to yarp::dev::IPositionControl interface is unexpectedly empty. Aborting operations."));
    }

    bool ret = false;

    if (!p_control_->checkMotionDone(&ret))
    {
        yError() << class_name_ + "::isMotionDone(). Error: cannot call IPositionControl::checkMotionDone() for the end-effector with name '" + ee_name_ + "'.";
        return {};
    }

    return ret;
}


bool CubJointControl::moveToStreaming(const Eigen::VectorXd& joints)
{
    if (!p_direct_)
    {
        yError() << class_name_ + "::moveToStreaming(). Error: p_direct_ not defined for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    if (joints.size() != joints_.size())
    {
        yError() << class_name_ + "::moveToStreaming(). Error: joints.size() is " + std::to_string(joints.size()) + " but should be equal to " + std::to_string(joints_.size()) + " for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    /* Convert to degrees. */
    Eigen::VectorXd joints_deg = joints * 180.0 / M_PI;

    /* Let's move. */
    p_direct_->setPositions(joints_deg.data());

    return true;
}


bool CubJointControl::configure(const yarp::os::Bottle& group)
{
    /* Check for mandatory parameters. */
    if (!utils::checkParameters({{"joint_local_port", "name"}}, "", group, "", utils::ParameterType::String, false) ||
        !utils::checkParameters({{"joint_axes_list", "joint_ports_list"}}, "", group, "", utils::ParameterType::String, true))
    {
        yError() << class_name_ + "::configure(). Error: cannot load mandatory configuration parameters for end-effector with name '" + ee_name_ + "'. See the errors above.";
        return false;
    }

    /* Extract the end-effector name. */
    ee_name_ = group.find("name").asString();

    /* Open the controlboardremapper device. */
    yarp::os::Property prop;
    prop.put("device", "remotecontrolboardremapper");

    /* Add joints list. */
    prop.addGroup("axesNames");
    yarp::os::Bottle& axes_names_bot = prop.findGroup("axesNames").addList();
    axes_names_bot = *group.find("joint_axes_list").asList();

    /* Add axes_names to the object. */
    for (size_t i = 0; i < axes_names_bot.size(); ++i)
    {
        joints_.push_back(axes_names_bot.get(i).asString());
    }

    /* Add remote control boards. */
    prop.addGroup("remoteControlBoards");
    yarp::os::Bottle& rcb_bot = prop.findGroup("remoteControlBoards").addList();
    rcb_bot = *group.find("joint_ports_list").asList();

    /* Add local port to property. */
    prop.put("localPortPrefix", group.find("joint_local_port").asString());

    /* Open the polydriver. */
    if (!drv_.open(prop))
    {
        yError() << class_name_ + "::configure(). Error: cannot open remotecontrolboardremapper for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    if (!drv_.view(p_control_))
    {
        yError() << class_name_ + "::configure(). Error: cannot open the IPositionControl view for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    if (!drv_.view(p_direct_))
    {
        yError() << class_name_ + "::configure(). Error: cannot open the IPositionDirect view for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    if (!drv_.view(control_limits_))
    {
        yError() << class_name_ + "::configure(). Error: cannot open the IControlLimits for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    if (!drv_.view(control_mode_))
    {
        yError() << class_name_ + "::configure(). Error: cannot open the IControlMode for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    if (!drv_.view(encoders_))
    {
        yError() << class_name_ + "::configure(). Error: cannot open the IEncoders for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    /* Backup control modes and reference speeds at configuration time. */
    joints_modes_original_.resize(joints_.size());
    if (!getControlModes(joints_modes_original_))
    {
        yError() << class_name_ + "::configure(). Cannot backup the current joints control modes for the end-effector with name '" + ee_name_ + "'.";
        return false;
    }

    /* yarp::dev::IPositionControl::getRefSpeeds() doesn't work in simulation. See https://github.com/hsp-iit/grasping-baselines/issues/77#issue-2014469217 */
    joints_speeds_original_.resize(joints_.size());
    for (int i = 0; i < joints_.size(); ++i)
    {
        if (!p_control_->getRefSpeed(i, i + joints_speeds_original_.data()))
        {
            yError() << class_name_ + "::configure(). Cannot backup the current joints speeds for the end-effector with name '" + ee_name_ + "'.";
            return false;
        }
    }

    if (!configureJointsMode("NonStreaming"))
    {
        yError() << class_name_ + "::configure(). Error: Cannot set 'NonStreaming' control mode. See the errors above.";
        return false;
    }
    joints_modes_current_ = VOCAB_CM_POSITION;

    return true;
}


bool CubJointControl::configureJointsMode(const std::string& mode)
{
    yarp::conf::vocab32_t des_mode;

    if (mode == "NonStreaming")
    {
        des_mode = VOCAB_CM_POSITION;
    }
    else if (mode == "Streaming")
    {
        des_mode = VOCAB_CM_POSITION_DIRECT;
    }
    else
    {
        yError() << class_name_ + "::configureJointsMode(). Error: unknown mode for the end-effector with name '" + ee_name_ + "'. Use Streaming or NonStreaming.";
        return false;
    }

    /* Retrieve current control mode. */
    std::vector<int> current_modes(joints_.size());
    if (!getControlModes(current_modes))
    {
        yError() << class_name_ + "::configureJointsMode(). Error: cannot get the current joints control modes of the end-effector with name '" + ee_name_ + "' using IControlMode::getControlModes().";
        return false;
    }

    /* Sanity check. */
    bool set_joints = false;
    for (int i = 0; i < current_modes.size(); ++i)
    {
        if ((current_modes[i] == VOCAB_CM_HW_FAULT) or (current_modes[i] == VOCAB_CM_IDLE))
        {
            yError() << class_name_ + "::configureJointsMode(). Error: Cannot set desired control mode. At least one joint of the end-effector with name '" + ee_name_ + "' is in HW_FAULT or in IDLE.";

            joints_modes_current_ = VOCAB_CM_HW_FAULT;

            return false;
        }

        if (current_modes[i] != des_mode)
        {
            /* Remember if at least one joint was not in the desired control mode. */
            set_joints = true;
        }
    }

    /* Set all joints again for simplicity. */
    if (set_joints)
    {
        if(!setMode(mode))
        {
            yError() << class_name_ + "::configureJointsMode(). Error: Cannot set desired control mode for the joints of the end-effector with name '" + ee_name_ + "'. See errors above.";
            return false;
        }
    }

    return true;
}


bool CubJointControl::getControlModes(std::vector<int>& joint_modes)
{
    bool success = false;
    int max_attempt = 10;
    for (int attempts = 1; attempts <= max_attempt; attempts++)
    {
        success = control_mode_->getControlModes(joint_modes.data());
        if (success)
            break;
        yarp::os::Time::delay(0.1);
    }

    return success;
}


std::optional<Eigen::VectorXd> CubJointControl::getJointValues() const
{
    /* Get current joint values */
    Eigen::VectorXd joints;

    joints.resize(joints_.size());

    if (!encoders_->getEncoders(joints.data()))
        return {};

    joints *= M_PI / 180.0;

    return joints;
}


bool CubJointControl::getJointValues2(Eigen::VectorXd& joints) const
{
    joints.resize(joints_.size());

    if (!encoders_->getEncoders(joints.data()))
    {
        return false;
    }

    joints *= M_PI / 180.0;

    return true;
}


std::optional<Eigen::VectorXd> CubJointControl::getJointRefValues() const
{

    if(joints_modes_current_ != VOCAB_CM_POSITION_DIRECT)
    {
        yError()<< class_name_ + "::getJointRefValues(). Error: Cannot get joint reference values for the end-effector with name '" + ee_name_ + " because current mode is NOT 'Streaming'.";
        return {};
    }

    if (!p_direct_)
    {
        yError() << class_name_ + "::getJointRefValues(). Error: p_direct_ not defined for the end-effector with name '" + ee_name_ + "'.";
        return {};
    }

    /* Get current joint values */
    Eigen::VectorXd joints;

    joints.resize(joints_.size());

    bool prova=true;

    for (size_t i = 0; i < joints.size(); i++)
    {
        prova &= p_direct_->getRefPosition(i, &joints[i]);
    }

    if (!prova)
        return {};

    joints *= M_PI / 180.0;

    return joints;
}


std::optional<Eigen::VectorXd> CubJointControl::getJointSpeeds() const
{
    /* Get current joint speeds */
    Eigen::VectorXd joints;

    joints.resize(joints_.size());
    if (!encoders_->getEncoderSpeeds(joints.data()))
        return {};

    joints *= M_PI / 180.0;

    return joints;
}


std::optional<Eigen::VectorXd> CubJointControl::getJointAccelerations() const
{
    /* Get current joint speeds */
    Eigen::VectorXd joints;

    joints.resize(joints_.size());
    if (!encoders_->getEncoderAccelerations(joints.data()))
        return {};

    joints *= M_PI / 180.0;

    return joints;
}


std::optional<std::unordered_map<std::string, Eigen::VectorXd>> CubJointControl::getJointLimits() const
{
    /* Get joint limits */
    Eigen::VectorXd upper(joints_.size());
    Eigen::VectorXd lower(joints_.size());

    for (int i = 0; i < joints_.size(); ++i)
        if (!control_limits_->getLimits(i, &lower[i], &upper[i]))
            return {};

    upper *= M_PI / 180.0;
    lower *= M_PI / 180.0;

    std::unordered_map<std::string, Eigen::VectorXd> limit_map;
    limit_map["lower"] = lower;
    limit_map["upper"] = upper;

    return limit_map;
}
