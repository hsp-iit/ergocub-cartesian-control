#include <module.h>

#include <yarp/os/LogStream.h>
#include <yarp/eigen/Eigen.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <inverseKinematicsParameters.h>

bool Module::configure(yarp::os::ResourceFinder& rf)
{
    /* Get robot name. */
    if (!(rf.check("robot") && rf.find("robot").isString()))
    {
        yError() << module_name_ + "::configure. Error: mandatory parameter 'robot' missing or invalid.";
        return false;
    }
    const std::string robot_name = rf.find("robot").asString();

    /* Get robot urdf path. */
    const std::string robot_urdf_path = rf.findFileByName("model.urdf");
    if (robot_urdf_path.empty())
    {
        yError() << module_name_ + "::configure. Error: cannot load the robot urdf path. Please check that the YARP_ROBOT_NAME environment variable is set.";
        return false;
    }

    /* Get module running rate. */
    if (!(rf.findGroup("COMMON").check("rate") && rf.findGroup("COMMON").find("rate").isFloat64()))
    {
        yError() << module_name_ + "::configure. Error: mandatory parameter 'rate' missing or invalid.";
        return false;
    }
    rate_ = rf.findGroup("COMMON").find("rate").asFloat64();


    /* Get print flag. */
    if (!(rf.findGroup("COMMON").check("print") && rf.findGroup("COMMON").find("print").isBool()))
    {
        yError() << module_name_ + "::configure. Error: mandatory parameter 'print' missing or invalid.";
        return false;
    }
    print_ = rf.findGroup("COMMON").find("print").asBool();

    /* Configure robot I/O */
    if (!robot_.configure(robot_name, list_actuated_joints_, "/" + module_name_))
    {
        yError() << module_name_ + "::configure. Error: cannot initialize the component for input/output communication with the robot.";
        return false;
    }

    /* Configure forward kinematics. */
    if (!fk_.configure(robot_urdf_path, robot_.getJointsList(), robot_.getActuatedJointsList(), "root_link", "head"))
    {
        yError() << module_name_ + "::configure. Error: cannot configure the forward kinematics.";
        return false;
    }

    /* Read encoders and initialize related variables. */
    if(!updateEncoders())
    {
        yError() << module_name_ + "::configure. Error: cannot read encoders.";
        return false;
    }
    meas_.q_na_prev = meas_.q_na;
    meas_.dq_na = Eigen::VectorXd::Zero(meas_.q_na.size());

    ref_.q_a = meas_.q_a;

    /* Initial desired orientation. */
    fk_.setJoints(meas_.q_na, meas_.q_a);
    fk_.update();
    R_desired_ = fk_.getTransform().rotation();

    /* Configure joint limits. */
    const auto limits = robot_.getLimitsActuatedJoints();
    if (!limits.has_value())
    {
        yError() << module_name_ + "::configure. Error: cannot retrieve joint limits.";
        return false;
    }

    /* Configure inverse kinematics. */
    InverseKinematicsParameters ik_parameters;
    if (!ik_parameters.configure(rf.findGroup("IK_PARAMETERS")))
    {
        yError() << module_name_ + "::configure. Error: cannot retrieve the inverse kinematics parameters. ";
        return false;
    }
    ik_.configure(robot_.getNumberActuatedJoints(), ik_parameters, getPeriod());
    ik_.setLimits(limits->lower, limits->upper);


    /* Configure RPC service. */
    if (!configureService(rf))
    {
        yError() << module_name_ + "::configure. Error: cannot configure the RPC service.";
        return false;
    }

    return true;
}


bool Module::close()
{
    if (port_rpc_.isOpen())
        port_rpc_.close();

    return true;
}


double Module::getPeriod()
{
    return 1.0 / rate_;
}


bool Module::interruptModule()
{
    return true;
}


bool Module::updateModule()
{
    /* Read encoders. */
    if(     !updateEncoders()
        ||  !updateForwardKinematics())
    {
        yError() << module_name_ + "::updateModule(). See error(s) above. ";
        return false;
    }

    /* Fetch current state. */
    Module::State state;
    {
        std::lock_guard<std::mutex> lg(mutex_);
        state = state_;
    }

    if (state == Module::State::Running)
    {
        {
            /* Protect this section as it depends on parameters changed via RPC calls. */
            std::lock_guard<std::mutex> lg(mutex_);

            /* Update inverse kinematics. */
            ik_.setGazerState(ref_.q_a, meas_.q_na, meas_.dq_na, fk_.getJacobianActuatedJoints(), fk_.getJacobianUnactuatedJoints(), fk_.getTransform());
            ik_.setDesiredGazingOrientation(R_desired_);
        }

        /* The following section is not protected as the ik_.update() call might not return,
           this way we are able to safely stop the robot from RPC. */
           ik_.update();

        /* Command the robot if there is a solution of the inverse kinematics. */
        const auto qd_a = ik_.getReferenceJointsVelocity();
        bool error = false;
        if (!qd_a.has_value())
        {
            yWarning() << module_name_ + "::updateModule(). Warning: cannot find a feasible solution.";
        }
        else
        {
            /* Check if the requested target is safe, i.e., not too far from the current joints configuration. */
            Eigen::VectorXd absolute_shift = ((*qd_a) * getPeriod()).array().abs();
            for (std::size_t i = 0; i < absolute_shift.size(); i++)
            {
                if (absolute_shift(i) > 10.0 * M_PI / 180.0)
                    error = true;
            }

            if (error)
                yWarning() << module_name_ + "::updateModule(). Error: the new set point is not safe.";
            else
            {
                /* Integrate the solution. */
                ref_.q_a += (*qd_a) * getPeriod();
                if (!robot_.moveActuatedJoints(ref_.q_a))
                {
                    yError() << module_name_ + "::updateModule(). Error: cannot call IO::moveActuatedJoints. Stopping the controller.";
                    std::lock_guard<std::mutex> lg(mutex_);
                    state_ = Module::State::Idle;
                }
            }
        }
    }

    if(print_) print();

    return true;
}


bool Module::updateEncoders()
{
    std::lock_guard<std::mutex> lg(mutex_);

    /* Read encoders. */
    auto encoders = robot_.getEncodersAllJoints();
    while (!encoders.has_value()) {
        yError() << module_name_ + "::updateEncoders(). Trying to read encoders...";
        encoders = robot_.getEncodersAllJoints();
    }

    /* Update actuated joints. */
    meas_.q_a = encoders->at("actuated");

    /* Update non actuated joints. */
    if (encoders->count("non-actuated") > 0)
    {
        meas_.q_na_prev = meas_.q_na;
        meas_.q_na = encoders->at("non-actuated");
        meas_.dq_na = (meas_.q_na - meas_.q_na_prev) / getPeriod();
    }

    return true;
}


bool Module::updateForwardKinematics()
{
    std::lock_guard<std::mutex> lg(mutex_);

    if(!fk_.setJoints(meas_.q_na, ref_.q_a))
    {
        yError() << module_name_ + "::updateForwardKinematics(). Error: cannot set joints.";
        return false;
    }

    fk_.update();
    
    return true;
}


void Module::print()
{
    /* Fetch protected variables. */
    Module::State state;
    Eigen::Matrix3d cur_ori;
    {
        std::lock_guard<std::mutex> lg(mutex_);
        state = state_;
        cur_ori = fk_.getTransform().rotation();
    }

    yInfo() << "---------- HEAD CONTROLLER -----------";
    if (state == Module::State::Idle)
        {yInfo() << "state\t|Idle|";}
    else if (state == Module::State::Running)
        yInfo() << "state\t|Running|";
    else
        yInfo() << "state\t|Unknown|";

    yInfo() << "---------- JOINT SPACE -----------";
    yInfo()<<"cur Nact joints"<<meas_.q_na.matrix();
    yInfo()<<"cur  act joints"<<meas_.q_a.matrix();
    yInfo()<<"ref  act joints"<<ref_.q_a.matrix();
    yInfo()<<"err  act joints"<<(ref_.q_a - meas_.q_a).matrix();


    Eigen::Matrix3d ref_ori;
    {
        std::lock_guard<std::mutex> lg(mutex_);
        fk_.setJoints(meas_.q_na, ref_.q_a);
        fk_.update();
        ref_ori = fk_.getTransform().rotation();
    }
    yInfo() << "---------- CARTESIAN SPACE -----------";
    auto cur_ori_aa = Eigen::AngleAxisd(cur_ori);
    auto ref_ori_aa = Eigen::AngleAxisd(ref_ori);
    auto des_ori_aa = Eigen::AngleAxisd(R_desired_);
    auto err_ori_aa = Eigen::AngleAxisd(ref_ori * cur_ori.transpose());
    yInfo()<<"cur ori - axis: "<<cur_ori_aa.axis() << "\t angle: " << cur_ori_aa.angle();
    yInfo()<<"ref ori - axis: "<<ref_ori_aa.axis() << "\t angle: " << ref_ori_aa.angle();
    yInfo()<<"tar ori - axis: "<<des_ori_aa.axis() << "\t angle: " << des_ori_aa.angle();
    yInfo()<<"err ori - axis: "<<err_ori_aa.axis() << "\t angle: " << err_ori_aa.angle();

    yInfo() << "################################################################";
}
