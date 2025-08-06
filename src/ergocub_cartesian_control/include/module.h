// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MODULE_H
#define MODULE_H

#include <Eigen/Dense>

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Port.h>

#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

#include <ForwardKinematicsiDynTree.h>
#include <DifferentialInverseKinematicsQP.h>
#include <Integrator.h>

#include <memory>
#include <string.h>
#include <mutex>

#include <cub-joint-control/cubJointControl.h>

#include <trajectory-generator/positionTrajectoryGenerator.h>
#include <trajectory-generator/orientationTrajectoryGenerator.h>

#include <gb-ergocub-cartesian-service/ergoCubCartesianService.h>

class Module : public yarp::os::RFModule,
               public ergoCubCartesianService
{
public:
    bool configure(yarp::os::ResourceFinder &rf) override;

    bool attach(yarp::os::Port &source) override;

    bool close() override;

    double getPeriod() override;

    bool interruptModule() override;

    bool updateModule() override;

    /* CartesianService interface */
    bool go_to_pose(double x, double y, double z, double q_x, double q_y, double q_z, double q_w, double traj_duration);

    bool go_to_position(double x, double y, double z, double traj_duration);

    bool rotate_deg(double angle, double x, double y, double z, double traj_duration);

    bool rotate_rad(double angle, double x, double y, double z, double traj_duration);

    yarp::sig::Matrix get_pose();

    bool go_home();

    bool is_motion_done();

    bool ask_reachability_evaluation(const yarp::sig::Matrix &pose);

    bool is_pose_reachable(const double x, const double y, const double z, const double q_x, const double q_y, const double q_z, const double q_w);

    yarp::sig::Matrix retrieve_reachable_pose();

    bool stop();

    bool setJointsMode(const std::string &mode) override;

private:
    /* Common variables */
    double rate_;
    double sample_time_;
    bool module_logging_;
    bool module_verbose_;

    Eigen::Transform<double, 3, Eigen::Affine> home_pose_;
    Eigen::Transform<double, 3, Eigen::Affine> desired_transform_;
    Eigen::Transform<double, 3, Eigen::Affine> current_transform_;
    Eigen::Vector3d desired_lin_vel_;
    Eigen::Vector3d desired_ang_vel_;

    void setCurrPose(const Eigen::Transform<double, 3, Eigen::Affine> &curr_pose);
    Eigen::Transform<double, 3, Eigen::Affine> getCurrPose();

    std::optional<Eigen::VectorXd> encoders_pos_, encoders_vel_, encoders_acc_, joint_home_values_;

    double max_joint_position_variation_;
    double max_joint_position_track_error_;

    /* Protects sections that depend on parameters read/changed via RPC calls */
    std::mutex mutex_;

    /* Trajectory */
    struct TRAJECTORY
    {
        bool is_ended;
        double duration;
        Eigen::Transform<double, 3, Eigen::Affine> fin_pose;
        Eigen::Transform<double, 3, Eigen::Affine> init_pose;
        std::unique_ptr<PositionTrajectory> lin_gen;
        std::unique_ptr<OrientationTrajectory> ang_gen;
    } traj_;
    const double min_traj_duration_ = 3.0;
    void setTrajIsEnded(bool is_ended);
    bool getTrajIsEnded();
    void setDuration(double duration);
    double getDuration();
    void setTrajFinPose(const Eigen::Transform<double, 3, Eigen::Affine> &fin_pose);
    void setTrajFinPose(const Eigen::Matrix4d &fin_pose);
    Eigen::Transform<double, 3, Eigen::Affine> getTrajFinPose();

    /* FSM */
    enum class State
    {
        Stop = 1,
        TrajInit,
        Running,
        ReachEval,
        Error
    };
    bool is_reach_eval_;
    State state_;
    void setState(const State &des_state);
    State getState();
    double stop_vel_;
    State error_prev_state_;
    bool updateReferenceVelocities();
    bool checkInputAndUpdateIntegrator();

    /* Controller */
    CubJointControl cub_joint_control_;

    /* Inverse kinematics */
    std::unique_ptr<DifferentialInverseKinematicsQP> ik_;
    Eigen::VectorXd ik_joint_vel_;
    std::optional<Eigen::VectorXd> ik_joint_acc_;

    /* Forward kinematics */
    std::unique_ptr<ForwardKinematicsiDynTree> fk_;

    /* Integrator */
    std::unique_ptr<Integrator> vel2pos_integrator_;
    std::unique_ptr<Integrator> acc2vel_integrator_;

    /* Reachability */
    double pos_err_th_;
    int max_iter_;

    /* RPC command port */
    yarp::os::Port rpc_cmd_port_;

    /* Module name */
    const std::string module_name_ = "gb-ergocub-cartesian-control";

    /* Logging */
    void log();
    BipedalLocomotion::YarpUtilities::VectorsCollectionServer m_vectorsCollectionServer; /** Logger server. */

    /* Thrift service configuration */
    bool configureService(const yarp::os::ResourceFinder &rf, const std::string rpc_port_name);

    /* Helper function for CartesianService */
    void serviceTrajInit(bool is_reach_eval, const Eigen::Affine3d &target_pose, double traj_duration);
};

#endif /* MODULE_H */
