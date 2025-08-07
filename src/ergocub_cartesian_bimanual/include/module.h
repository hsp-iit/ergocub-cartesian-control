#ifndef MODULE_H
#define MODULE_H

#include <Eigen/Dense>

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Port.h>

#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

#include <memory>
#include <string.h>
#include <mutex>

#include <ForwardKinematicsiDynTree.h>
#include <cub-joint-control/cubJointControl.h>
#include <mc-ergocub-bimanual-service/ergoCubBimanualService.h>

#include <DifferentialInverseKinematicsQP.h>
#include <Integrator.h>

class Module : public yarp::os::RFModule,
               public ergoCubBimanualService
{
public:
    bool configure(yarp::os::ResourceFinder &rf) override;

    bool attach(yarp::os::Port &source) override;

    bool close() override;

    double getPeriod() override;

    bool interruptModule() override;

    bool updateModule() override;

    /* CartesianService interface */
    bool go_to_pose(double x, double y, double z, double q_x, double q_y, double q_z, double q_w, const std::string& arm);

    bool go_to_position(double x, double y, double z, const std::string& arm);

    bool rotate_deg(double angle, double x, double y, double z, const std::string& arm);

    bool rotate_rad(double angle, double x, double y, double z, const std::string& arm);

    yarp::sig::Matrix get_pose(const std::string& arm);

    bool go_home();

    bool stop();

private:
    /* Module name */
    const std::string module_name_ = "mc-ergocub-cartesian-bimanual";

    /* general.ini */
    double sample_time_;
    bool module_logging_;
    bool module_verbose_;
    bool use_torso_;
    yarp::os::Port rpc_cmd_port_;
    yarp::os::BufferedPort<yarp::sig::Vector> bp_cmd_port_;

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
    bool checkAndReadNewInputs();
    Eigen::Affine3d right_desired_pose_, left_desired_pose_;
    Eigen::Vector3d right_desired_lin_vel_, right_desired_ang_vel_, right_desired_lin_acc_, right_desired_ang_acc_;
    Eigen::Vector3d left_desired_lin_vel_, left_desired_ang_vel_, left_desired_lin_acc_, left_desired_ang_acc_;

    /* Protects sections that depend on parameters read/changed via RPC calls */
    std::mutex mutex_;

    /* Logging */
    void verboseAndLog();
    BipedalLocomotion::YarpUtilities::VectorsCollectionServer m_vectorsCollectionServer; /** Logger server. */

    /* HELPER function*/
    void appendEigen(Eigen::VectorXd &vec, const Eigen::VectorXd &vec_app);
    Eigen::VectorXd concatenateEigen(const Eigen::VectorXd &vec1, const Eigen::VectorXd &vec2);

    /* Thrift service configuration */
    bool configureService(const yarp::os::ResourceFinder &rf, const std::string rpc_port_name);
};

#endif /* MODULE_H */
