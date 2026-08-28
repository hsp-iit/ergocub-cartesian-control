#include <io.h>

#include <vector>
#include <string>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <yarp/os/LogStream.h>

namespace
{
std::vector<yarp::dev::SelectableControlModeEnum> toSelectableControlModes(const std::vector<int>& modes)
{
    std::vector<yarp::dev::SelectableControlModeEnum> yarpModes;
    yarpModes.reserve(modes.size());
    for (const int mode : modes)
    {
        yarpModes.push_back(static_cast<yarp::dev::SelectableControlModeEnum>(mode));
    }
    return yarpModes;
}

bool getControlModes(yarp::dev::IControlMode* mode,
                     const std::vector<int>& joints,
                     std::vector<int>& modes)
{
    std::vector<yarp::dev::ControlModeEnum> yarpModes(modes.size());
    const bool ok = mode->getControlModes(joints, yarpModes);
    if (!ok)
    {
        return false;
    }

    for (std::size_t i = 0; i < yarpModes.size(); ++i)
    {
        modes[i] = static_cast<int>(yarpModes[i]);
    }
    return true;
}
} // namespace


IO::~IO()
{
    /* Restore control mode of actuated joints. */
    if ((mode_ != nullptr) && (!mode_->setControlModes(actuated_joints_indexes_, toSelectableControlModes(actuated_joints_original_mode_))))
    {
        yError() << class_name_ + "::configure. Cannot restore the control mode of the actuated joints using IControlMode::setControlModes().";
    }

    /* Close the driver. */
    if (driver_.isValid())
        driver_.close();
}


bool IO::configure(const std::string& robot, const std::vector<std::string>& actuated_joints_list, const Eigen::VectorXd& actuated_joints_signs, const std::string& local_port_name)
{
    /* Store the list of actuated joints names. */
    actuated_joints_list_ = actuated_joints_list;
    actuated_joints_signs_ = actuated_joints_signs;

    /* Setup the list of actuated and non actuated joints indexes. */
    for (std::size_t i = 0; i < joints_list_.size(); i++)
    {
        if (std::find(actuated_joints_list.begin(), actuated_joints_list.end(), joints_list_[i]) != actuated_joints_list.end())
            actuated_joints_indexes_.push_back(i);
        else
            non_actuated_joints_indexes_.push_back(i);
    }

    /* Setup remotecontrolboardremapper. */
    yarp::os::Property prop;
    prop.put("device", "remotecontrolboardremapper");

    /* Add joints list. */
    prop.addGroup("axesNames");
    yarp::os::Bottle& axes_names = prop.findGroup("axesNames").addList();
    for (const auto& name : joints_list_)
        axes_names.addString(name);

    /* Add remote control boards. */
    const std::vector<std::string> boards_list = {"/" + robot + "/torso", "/" + robot + "/head"};
    prop.addGroup("remoteControlBoards");
    yarp::os::Bottle& remote_boards = prop.findGroup("remoteControlBoards").addList();
    for (const auto& name : boards_list)
        remote_boards.addString(name);

    /* Add local port to property. */
    prop.put("localPortPrefix", local_port_name);

    /* Open driver. */
    if (!driver_.open(prop))
    {
        yError() << class_name_ + "::configure. Error: cannot configure the remotecontrolboardremapper driver.";
        return false;
    }

    /* Get views. */
    if (!driver_.view(limits_))
    {
        yError() << class_name_ + "::configure. Error: cannot retrieve the IControlLimits view.";
        return false;
    }
    if (!driver_.view(encoders_))
    {
        yError() << class_name_ + "::configure. Error: cannot retrieve the IEncoders view.";
        return false;
    }
    if (!driver_.view(control_))
    {
        yError() << class_name_ + "::configure. Error: cannot retrieve the IPositionDirect view.";
        return false;
    }
    if (!driver_.view(mode_))
    {
        yError() << class_name_ + "::configure. Error: cannot retrieve the IControlMode view.";
        return false;
    }

    /* Backup control mode of actuated joints. */
    actuated_joints_original_mode_.resize(getNumberActuatedJoints());
    bool success = false;
    int max_attempt = 10;
    for (int attempts = 1; attempts <= max_attempt; attempts++)
    {
        success = getControlModes(mode_, actuated_joints_indexes_, actuated_joints_original_mode_);
        if (success)
            break;
        yarp::os::Time::delay(0.1);
    }
    if (!success)
    {
        yError() << class_name_ + "::configure. Cannot backup the current joints control modes of the actuated joints using IControlMode::getControlModes().";
        return false;
    }

    /* Set actuated joints in Position Direct mode. */
    std::vector<int> modes(getNumberActuatedJoints(), VOCAB_CM_POSITION_DIRECT);

    if (!mode_->setControlMode(actuated_joints_indexes_[0], VOCAB_CM_POSITION_DIRECT))
    {
        yError() << class_name_ + "::moveActuatedJoints************. Error: cannot set the control mode of joint " << actuated_joints_indexes_[0] << " using IControlMode::setControlMode().";
        return false;
    }
    if (!mode_->setControlMode(actuated_joints_indexes_[1], VOCAB_CM_POSITION_DIRECT))
    {
        yError() << class_name_ + "::moveActuatedJoints************. Error: cannot set the control mode of joint " << actuated_joints_indexes_[1] << " using IControlMode::setControlMode().";
        return false;
    }

    
    
    // if (!mode_->setControlModes(actuated_joints_indexes_, toSelectableControlModes(modes)))
    // {
    //     yError() << class_name_ + "::configure. Cannot set the control mode of the actuated joints using IControlMode::setControlModes().";
    //     return false;
    // }


    return true;
}


std::size_t IO::getNumberActuatedJoints()
{
    return actuated_joints_list_.size();
}


std::size_t IO::getNumberAllJoints()
{
    return joints_list_.size();
}


std::vector<std::string> IO::getJointsList()
{
    return joints_list_;
}


std::vector<std::string> IO::getActuatedJointsList()
{
    return actuated_joints_list_;
}


std::optional<int> IO::getActuatedJointIndexByName(const std::string& name)
{
    /* First find the index within the overall set of joints. */
    int index;
    for (std::size_t i = 0; i < joints_list_.size(); i++)
    {
        if (joints_list_[i] == name)
        {
            index = i;
            break;
        }
    }

    /* Find the index within the set of actuated joints. */
    for (std::size_t i = 0; i < actuated_joints_indexes_.size(); i++)
    {
        if (actuated_joints_indexes_[i] == index)
            return i;
    }

    return {};
}


std::optional<std::unordered_map<std::string, Eigen::VectorXd>> IO::getEncodersAllJoints()
{
    if (!encoders_)
    {
        yError() << class_name_ + "::getEncodersAllJoints. Error: the IEncoders interface pointer is not valid.";
        return {};
    }

    int n_axes;
    if (!encoders_->getAxes(&n_axes))
    {
        yError() << class_name_ + "::getEncodersAllJoints. Error: the IEncoders::getAxes() method cannot be called.";
        return {};
    }
    if (n_axes != getNumberAllJoints())
    {
        yError() << class_name_ + "::getEncodersAllJoints. Error: the number of encoders provided by IEncoders::getAxes() should be equal to " + std::to_string(getNumberAllJoints()) + ", while it is " + std::to_string(n_axes);
        return {};
    }
    Eigen::VectorXd encoders(n_axes);
    if (!encoders_->getEncoders(encoders.data()))
    {
        yError() << class_name_ + "::getEncodersAllJoints. Error: the IEncoders::getEncoders() method cannot be called.";
        return {};
    }
    encoders *= M_PI / 180.0;

    std::unordered_map<std::string, Eigen::VectorXd> encoders_map;
    if (non_actuated_joints_indexes_.size() > 0)
    {
        encoders_map["non-actuated"] = Eigen::VectorXd(non_actuated_joints_indexes_.size());
        for (std::size_t i = 0; i < non_actuated_joints_indexes_.size(); i++)
            encoders_map["non-actuated"][i] = encoders[non_actuated_joints_indexes_[i]];
    }
    encoders_map["actuated"] = Eigen::VectorXd(getNumberActuatedJoints());
    for (std::size_t i = 0; i < getNumberActuatedJoints(); i++)
    {
        encoders_map["actuated"][i] = actuated_joints_signs_[i] * encoders[actuated_joints_indexes_[i]];
    }

    return encoders_map;
}


std::optional<IO::Limits> IO::getLimitsActuatedJoints()
{
    if (!limits_)
    {
        yError() << class_name_ + "::getLimitsActuatedJoints. Error: the IControlLimits interface pointer is not valid.";
        return {};
    }

    Eigen::VectorXd limits_min(getNumberAllJoints());
    Eigen::VectorXd limits_max(getNumberAllJoints());
    for (std::size_t i = 0; i < getNumberAllJoints(); i++)
    {
        if (!limits_->getPosLimits(i, &limits_min[i], &limits_max[i]))
        {
            yError() << class_name_ + "::getLimitsActuatedJoints(). Error: the IControlLimits::getPosLimits() method cannot be called.";
            return {};
        }
    }
    limits_min *= M_PI / 180.0;
    limits_max *= M_PI / 180.0;

    Eigen::VectorXd limits_min_actuated(getNumberActuatedJoints());
    Eigen::VectorXd limits_max_actuated(getNumberActuatedJoints());
    for (std::size_t i = 0; i < getNumberActuatedJoints(); i++)
    {
        const int joint_index = actuated_joints_indexes_[i];
        if (actuated_joints_signs_[i] > 0.0)
        {
            limits_min_actuated[i] = limits_min[joint_index];
            limits_max_actuated[i] = limits_max[joint_index];
        }
        else
        {
            limits_min_actuated[i] = -limits_max[joint_index];
            limits_max_actuated[i] = -limits_min[joint_index];
        }
    }

    return Limits{.lower = limits_min_actuated, .upper = limits_max_actuated};
}


bool IO::moveActuatedJoints(const Eigen::VectorXd& joints)
{
    if (!control_)
    {
        yError() << class_name_ + "::moveActuatedJoints. Error: the IPositionDirect interface pointer is not valid.";
        return false;
    }

    /* Get control modes to check if there are joints in fault, idle or not in the desired control mode. */
    std::vector<int> current_modes(getNumberActuatedJoints());
    if (!getControlModes(mode_, actuated_joints_indexes_, current_modes))
    {
        yError() << class_name_ + "::moveActuatedJoints. Error: cannot get the current joints control modes of the actuated joints using IControlMode::getControlModes().";
        return false;
    }

    /* Check control modes. */
    bool set_joints = false;
    for (std::size_t i = 0; i < current_modes.size(); i++)
    {
        if ((current_modes[i] == VOCAB_CM_HW_FAULT) or (current_modes[i] == VOCAB_CM_IDLE))
        {
            /* If a joint is in fault or idle, do not actuate joints as a partial actuation will not be safe. */

            yError() << class_name_ + "::moveActuatedJoints. Error: some joints are in HW_FAULT or in IDLE. The request command will not be executed.";
            return false;
        }

        if (current_modes[i] != VOCAB_CM_POSITION_DIRECT)
        {
            /* Remember if at least one joint was not in the desired control mode. */
            set_joints = true;

            /* Altough we have found at least one joint to be set, do not break here as one of the next joints might in fault or idle. */
        }
    }
    if (set_joints)
    {
        /* Set all joints again for simplicity. */
        std::vector<int> modes(getNumberActuatedJoints(), VOCAB_CM_POSITION_DIRECT);

        if (!mode_->setControlMode(actuated_joints_indexes_[0], VOCAB_CM_POSITION_DIRECT))
        {
            yError() << class_name_ + "::moveActuatedJoints************. Error: cannot set the control mode of joint " << actuated_joints_indexes_[0] << " using IControlMode::setControlMode().";
            return false;
        }
        if (!mode_->setControlMode(actuated_joints_indexes_[1], VOCAB_CM_POSITION_DIRECT))
        {
            yError() << class_name_ + "::moveActuatedJoints************. Error: cannot set the control mode of joint " << actuated_joints_indexes_[1] << " using IControlMode::setControlMode().";
            return false;
        }

        // if (!mode_->setControlModes(actuated_joints_indexes_, toSelectableControlModes(modes)))
        // {
        //     yError() << class_name_ + "::moveActuatedJoints. Cannot set the control mode of the actuated joints using IControlMode::setControlModes().";
        //     return false;
        // }
    }


    const Eigen::VectorXd joints_hw = actuated_joints_signs_.array() * joints.array();
    Eigen::VectorXd joints_deg = joints_hw * 180.0 / M_PI;

    return control_->setPositions(getNumberActuatedJoints(), actuated_joints_indexes_.data(), joints_deg.data());
}
