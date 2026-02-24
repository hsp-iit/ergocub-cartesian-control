#ifndef MODULE_H
#define MODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

#include <forwardKinematics.h>
#include <inverseKinematics.h>
#include <io.h>
#include <yarp/sig/Matrix.h>

#include <mc-ergocub-head-service/ergoCubHeadService.h>

#include <mutex>


class Module : public yarp::os::RFModule,
               public ergoCubHeadService
{
public:
    bool configure(yarp::os::ResourceFinder& rf) override;

    bool close() override;

    double getPeriod() override;

    bool interruptModule() override;

    bool updateModule() override;

    void print();

    /**
     * ergoCubGazeService interface
     */
    bool setOrientation(const yarp::sig::Matrix& rot);

    bool setOrientationFlat(double r11, double r12, double r13, double r21, double r22, double r23, double r31, double r32, double r33);

    bool rotateAxisAngle(const double x, const double y, const double z, const double angle);

    bool goHome();

    bool stop();

private:
    enum class State {Idle, Running};

    /* Thrift service configuration. */
    bool configureService(const yarp::os::ResourceFinder& rf);
    bool checkAndReadRpcCommands();

    /* Robot input / output. */
    IO robot_;

    /* Forward kinematics. */
    ForwardKinematics fk_;
    bool updateForwardKinematics();

    /* Inverse kinematics. */
    InverseKinematics ik_;

    /* Desired orientation. */
    Eigen::Matrix3d R_desired_;

    /**
     * Please be *very* careful when changing this as including torso joints will cause
     * the controller to also use them to track the gazing point.
     */
    const std::vector<std::string> list_actuated_joints_ = {"neck_pitch", "neck_roll", "neck_yaw"}; //, "camera_tilt"

    /* Storage. */
    struct joint_storage{
        Eigen::VectorXd q_a, q_na, dq_na;
        Eigen::VectorXd q_na_prev;
    }meas_, ref_;
    bool updateEncoders();

    /* Module state. */
    State state_ = State::Idle;
    std::mutex mutex_;

    /* RPC command port. */
    yarp::os::BufferedPort<yarp::os::Bottle> rpc_cmd_port_;

    /* Module running rate. */
    double rate_;

    /* Print flag*/
    bool print_;

    /* Module name. */
    const std::string module_name_ = "mc-ergocub-head-controller";
};

#endif /* MODULE_H */
