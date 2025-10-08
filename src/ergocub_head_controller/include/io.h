#ifndef IO_H
#define IO_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IEncoders.h>

#include <Eigen/Dense>

#include <optional>


class IO
{
public:
    struct Limits
    {
        Eigen::VectorXd lower;
        Eigen::VectorXd upper;
    };

    ~IO();

    bool configure(const std::string& robot, const std::vector<std::string>& actuated_joints_list, const std::string& local_port_name);

    std::size_t getNumberActuatedJoints();

    std::size_t getNumberAllJoints();

    std::vector<std::string> getJointsList();

    std::vector<std::string> getActuatedJointsList();

    std::optional<int> getActuatedJointIndexByName(const std::string& name);

    std::optional<Limits> getLimitsActuatedJoints();

    std::optional<std::unordered_map<std::string, Eigen::VectorXd>> getEncodersAllJoints();

    bool moveActuatedJoints(const Eigen::VectorXd& joints);

private:
    yarp::dev::PolyDriver driver_;
    yarp::dev::IControlLimits* limits_;
    yarp::dev::IEncoders* encoders_;
    yarp::dev::IPositionDirect* control_;
    yarp::dev::IControlMode* mode_;

    /* List of joint names. */
    const std::vector<std::string> joints_list_ = {"torso_roll", "torso_pitch", "torso_yaw", "neck_pitch", "neck_roll", "neck_yaw"};    //, "camera_tilt"

    /* List of the actuated joints indexes and names. */
    std::vector<std::string> actuated_joints_list_;
    std::vector<int> actuated_joints_indexes_;

    /* List of the non actuated joints indexes and names. */
    std::vector<int> non_actuated_joints_indexes_;

    /* Backup of actuated joints control modes at configuration time. */
    std::vector<int> actuated_joints_original_mode_;

    /* Class name. */
    const std::string class_name_ = "IO";
};

#endif /* IO_H */
