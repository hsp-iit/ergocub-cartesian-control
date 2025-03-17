#ifndef JOINT_CONTROL_H
#define JOINT_CONTROL_H

#include <optional>

#include <Eigen/Dense>
#include <yarp/os/Bottle.h>


class JointControl
{
public:
    virtual ~JointControl() {};

    /**
     * Get the number of joints.
     * @return An std::size_t containing the number of joints.
     */
    virtual std::size_t getNumberJoints() = 0;

    /**
    * Switch control mode.
    * @param mode A string containing the control mode of the joints.
    * @return True if mode is correctly set, False otherwise.
    */
    virtual bool setMode(const std::string& mode) = 0;

    /**
     * Set speeds used in JointControl::moveTo().
     * This is to be used only together with JointControl::moveTo().
     * @return True if the speeds are set, False otherwise.
     */
    virtual bool setSpeeds(const Eigen::VectorXd& speeds) = 0;

    /**
     * Stop the motion of the joint.
     * @return True if chain is successful, False otherwise.
     */
    virtual bool stop() = 0;

    /**
     * Move the chain to a given configuration.
     * @param joints A Eigen::VectorXd containing the target joint configuration.
     * @param blocking Whether the call is to be blocking or not. A blocking call is
     *                 expected to return when the motion is completed.
     * @return Nothing if there is an error, True if motion is done, False otherwise.
     */
    virtual bool moveTo(const Eigen::VectorXd& joints, const bool& blocking) = 0;

    /**
     * Retrieve motion completion status.
     * This is to be used only together with JointControl::moveTo().
     * @return Nothing if an error occurs, True if the motion is completed, False otherwise.
     */
    virtual std::optional<bool> isMotionDone() = 0;

    /**
     * Move the chain to a given configuration in "streaming" mode, meaning that
     * it is the user responsability to provide a suitable trajectory over time
     * with a proper sampling time.
     * @param joints A Eigen::VectorXd containing the next set point within the joint trajectory.
     * @return True if the motion request is accepted, False otherwise.
     */
    virtual bool moveToStreaming(const Eigen::VectorXd& joints) = 0;

    /**
     * Configure the joint controller. Check the specific implementation documentation to
     * know more about the mandatory/optional parameters.
     * @param group A yarp::os::Bottle containing the configuration of the end-effector for which the joint controller is to be configured.
     * @return True if the configuration is accepted, False otherwise.
    */
   virtual bool configure(const yarp::os::Bottle& group) = 0;
};

#endif
