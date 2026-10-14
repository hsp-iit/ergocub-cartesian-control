#ifndef MODULE_H
#define MODULE_H

#include <Eigen/Dense>

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>

#ifdef BIMANUAL_WITH_WALKING_RETARGETING
#include <WalkingControllers/YarpUtilities/HumanState.h>
#endif

#include <atomic>
#include <memory>
#include <string>
#include <string.h>
#include <mutex>
#include <vector>

#include <ergocub_cartesian_bimanual/ForwardKinematicsiDynTree.h>
#include <cub-joint-control/cubJointControl.h>

#include <ergocub_cartesian_bimanual/DifferentialInverseKinematicsQP.h>
#include <ergocub_cartesian_bimanual/Integrator.h>
#include <ergocub_cartesian_bimanual/SelfCollision.h>

class Module : public yarp::os::RFModule
{
public:
    explicit Module(const std::string &module_name);

    bool configure(yarp::os::ResourceFinder &rf) override;

    bool close() override;

    double getPeriod() override;

    bool interruptModule() override;

    bool updateModule() override;

    /* RPC interface (rpc_port_name) */
    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply) override;

    /* Query interface */
    bool go_to_position(double x, double y, double z, const std::string& arm);

    bool rotate_deg(double angle, double x, double y, double z, const std::string& arm);

    bool rotate_rad(double angle, double x, double y, double z, const std::string& arm);

    bool go_home();

    bool stop();
    
    bool right_enabled_{false};
    bool left_enabled_{false};
    bool torso_enabled_{false};

private:
    /* Module name */
    const std::string module_name_{"no-name"};

    /* general.ini */
    double sample_time_{};
    bool module_logging_{};
    bool module_verbose_{};
    bool use_torso_{};
    yarp::os::BufferedPort<yarp::os::Bottle> query_port_{};
    yarp::os::BufferedPort<yarp::sig::Vector> input_cmd_{}, joints_pos_port_{};
    bool no_control_{false};
    yarp::os::RpcServer rpc_port_;

    /* Retargeting to the walking-controller: joint references are sent to it instead of the motors.
     * The RPC thread only sets the requested mode, the switch happens in updateModule. */
    std::atomic<bool> retargeting_requested_{false};
    std::atomic<bool> retargeting_active_{false};
    bool applyRetargetingRequest();
#ifdef BIMANUAL_WITH_WALKING_RETARGETING
    yarp::os::BufferedPort<WalkingControllers::YarpUtilities::HumanState> retargeting_port_;
    std::vector<std::string> retargeting_joint_names_;
    std::string retargeting_remote_port_name_;
    double retargeting_last_reconnect_{0.0};
    bool connectRetargetingPort();
    bool publishToWalkingController(const Eigen::VectorXd &joint_refs);
#endif

    /* Forward kinematics */
    struct SUBCHAINS
    {
        std::optional<Eigen::VectorXd> joint_pos, joint_vel, joint_acc;
        CubJointControl cjc;
    } torso_, right_arm_, left_arm_;
    struct JOINTSTATE
    {
        Eigen::VectorXd pos, vel, acc;
    };
    struct CHAINS
    {
        JOINTSTATE measJoints, refJoints;
        std::unique_ptr<ForwardKinematicsiDynTree> measFk, refFk;
        Eigen::Affine3d home_pose;
        /* Upper-arm direction task (optional): FK of the upper-arm link on the reference joints. */
        std::unique_ptr<ForwardKinematicsiDynTree> upperArmFk;
        Eigen::Vector3d upper_arm_dir_local = -Eigen::Vector3d::UnitZ();
        Eigen::Vector3d desired_arm_dir = Eigen::Vector3d::Zero();
    } right_chain_, left_chain_;
    bool encodersMeasUpdate();
    bool encodersRefUpdate();
    bool measFksUpdate();
    bool refFksUpdate();
    bool moveChains();

    /* Inverse kinematics */
    struct IK
    {
        double stop_vel = 1000.0;
        JOINTSTATE joints;
        std::unique_ptr<DifferentialInverseKinematicsQP> ik;
        std::unique_ptr<Integrator> vel2pos_integrator, acc2vel_integrator;
    }compound_chain_;
    std::optional<Eigen::VectorXd> qp_result_;
    /* Self-collision avoidance (optional): QP constraints. The port publishes [smallest model distance [m], 0 ok | 1 softened | 2 braking]. */
    std::unique_ptr<SelfCollision> self_collision_;
    yarp::os::BufferedPort<yarp::sig::Vector> self_collision_port_{};
    void ikUpdate();
    void setDesiredTrajectory();
    bool solveIkAndUpdateIntegrators();

    /* FSM */
    enum class State
    {
        Stop = 1,
        Running
    }state_;
    void setState(const State &des_state);
    State getState();
    bool isMotionDone();

    /* Control input*/
    bool checkAndReadQuery();
    bool checkAndReadNewInputs();
    void checkAndReadArmDir();
    yarp::os::BufferedPort<yarp::sig::Vector> arm_dir_port_{};
    double arm_dir_timeout_{0.5};
    double arm_dir_last_time_{-1.0};
    Eigen::Affine3d right_desired_pose_, left_desired_pose_;
    Eigen::Vector3d right_desired_lin_vel_, right_desired_ang_vel_, right_desired_lin_acc_, right_desired_ang_acc_;
    Eigen::Vector3d left_desired_lin_vel_, left_desired_ang_vel_, left_desired_lin_acc_, left_desired_ang_acc_;

    /* Protects state */
    std::mutex mutex_;

    /* Logging */
    void verboseAndLog();

    /* HELPER function*/
    void appendEigen(Eigen::VectorXd &vec, const Eigen::VectorXd &vec_app);
    Eigen::VectorXd concatenateEigen(const Eigen::VectorXd &vec1, const Eigen::VectorXd &vec2);
};

#endif /* MODULE_H */
