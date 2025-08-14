// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef CUB_JOINT_CONTROL_H
#define CUB_JOINT_CONTROL_H

#include <cub-joint-control/jointControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>

#include <optional>

#include <Eigen/Dense>

class CubJointControl : public JointControl
{
public:
    CubJointControl() = default;
    ~CubJointControl();
    std::size_t getNumberJoints() override;
    bool setMode(const std::string& mode) override;
    bool setSpeeds(const Eigen::VectorXd& speeds) override;
    bool stop() override;
    bool moveTo(const Eigen::VectorXd& joints, const bool& blocking) override;
    std::optional<bool> isMotionDone() override;
    bool moveToStreaming(const Eigen::VectorXd& joints) override;
    bool configure(const yarp::os::Bottle& group);

    /* Configure control mode.*/
    bool configureJointsMode(const std::string& mode);

    /* Retrieve control mode with attempts*/
    bool getControlModes(std::vector<int>& joint_modes);

    /* Retrieve current joint values. */
    std::optional<Eigen::VectorXd> getJointValues() const;
    bool getJointValues2(Eigen::VectorXd& joints) const override;

    /* Retrieve joint reference values. */
    std::optional<Eigen::VectorXd> getJointRefValues() const;

    /* Retrieve joint speeds. */
    std::optional<Eigen::VectorXd> getJointSpeeds() const;

    /* Retrieve joint accelerations. */
    std::optional<Eigen::VectorXd> getJointAccelerations() const;

    /* Retrieve upper and lower joints limits. */
    std::optional<std::unordered_map<std::string, Eigen::VectorXd>> getJointLimits() const;

private:
    yarp::dev::PolyDriver drv_;
    yarp::dev::IPositionDirect* p_direct_;
    yarp::dev::IPositionControl* p_control_;
    yarp::dev::IControlLimits* control_limits_;
    yarp::dev::IControlMode* control_mode_;
    yarp::dev::IEncoders* encoders_;

    std::vector<std::string> joints_;
    int joints_modes_current_ = VOCAB_CM_UNKNOWN;

    /* Backup variables for the original context. */
    std::vector<int> joints_modes_original_;
    std::vector<double> joints_speeds_original_;

    std::string ee_name_;

    std::string class_name_ = "CubJointControl";
};

#endif
