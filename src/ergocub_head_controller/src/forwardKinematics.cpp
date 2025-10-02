#include <forwardKinematics.h>

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/EigenHelpers.h>

#include <yarp/os/LogStream.h>


using namespace Eigen;
using namespace iDynTree;


bool ForwardKinematics::configure(const std::string& urdf_path, const std::vector<std::string>& joints_list, const std::vector<std::string>& actuated_joints_list, const std::string& root_frame_name, const std::string& ee_frame_name)
{
    /* Setup the list of actuated and non actuated joints indexes. */
    for (std::size_t i = 0; i < joints_list.size(); i++)
    {
        if (std::find(actuated_joints_list.begin(), actuated_joints_list.end(), joints_list[i]) != actuated_joints_list.end())
            actuated_joints_indexes_.push_back(i);
        else
            non_actuated_joints_indexes_.push_back(i);
    }

    /* Load the model. */
    ModelLoader loader;
    if (!loader.loadReducedModelFromFile(urdf_path, joints_list))
    {
        yError() << class_name_ + "::configure(). Error: cannot load the URDF model from path " + urdf_path;
        return false;
    }

    /* Instantiate the chain. */
    chain_.loadRobotModel(loader.model());

    /* Check that the frames of interest do exist. */
    root_frame_idx_ = chain_.getFrameIndex(root_frame_name);
    ee_frame_idx_ = chain_.getFrameIndex(ee_frame_name);

    if (root_frame_idx_ < 0)
        yError() << class_name_ + "::configure(). Error: cannot find root frame " + root_frame_name + " in model " + urdf_path;
    if (ee_frame_idx_ < 0)
        yError() << class_name_ + "::configure(). Error: cannot find end-effector frame " + ee_frame_name + " in model " + urdf_path;
    if ((root_frame_idx_ < 0) || (ee_frame_idx_ < 0))
        return false;

    /* Set the mixed twist representation.
       It means that:
       - the linear velocity is that of the origin of the end-effector frame expressed in the root frame;
       - the angular velocity is that of the end-effector frame expressed in the root frame.
    */
    chain_.setFrameVelocityRepresentation(FrameVelocityRepresentation::MIXED_REPRESENTATION);

    /* Resize the jacobian. */
    jacobian_.resize(6, chain_.getNrOfDegreesOfFreedom());

    return true;
}


void ForwardKinematics::update()
{
    /* End-effector transform. */
    chain_.getWorldTransform(ee_frame_idx_, transform_.matrix());

    /* Jacobian. */
    chain_.getRelativeJacobian(root_frame_idx_, ee_frame_idx_, jacobian_);
}


bool ForwardKinematics::setJoints(const Eigen::VectorXd& joints_non_actuated, const Eigen::VectorXd& joints_actuated)
{
    std::size_t number_actuated = actuated_joints_indexes_.size();
    std::size_t number_non_actuated = non_actuated_joints_indexes_.size();

    if (joints_non_actuated.size() != number_non_actuated)
    {
        yError() << class_name_ + "::setJoints. Error: 'joints_non_actuated' should have size equal to " + std::to_string(number_non_actuated) + ".";
        return false;
    }

    if (joints_actuated.size() != number_actuated)
    {
        yError() << class_name_ + "::setJoints. Error: joints_actuated should have size equal to "  + std::to_string(number_actuated) + ".";
        return false;
    }

    Eigen::VectorXd joints(chain_.getNrOfDegreesOfFreedom());
    for (std::size_t i = 0; i < non_actuated_joints_indexes_.size(); i++)
        joints[non_actuated_joints_indexes_[i]] = joints_non_actuated[i];
    for (std::size_t i = 0; i < actuated_joints_indexes_.size(); i++)
        joints[actuated_joints_indexes_[i]] = joints_actuated[i];

    chain_.setJointPos(joints);

    return true;
}


Eigen::Transform<double, 3, Affine> ForwardKinematics::getTransform()
{
    return transform_;
}


MatrixXd ForwardKinematics::getJacobian()
{
    return jacobian_;
}


Eigen::MatrixXd ForwardKinematics::getJacobianActuatedJoints()
{
    Eigen::MatrixXd J(jacobian_.rows(), actuated_joints_indexes_.size());
    for (std::size_t i = 0; i < actuated_joints_indexes_.size(); i++)
        J.col(i) = jacobian_.col(actuated_joints_indexes_[i]);

    return J;
}


Eigen::MatrixXd ForwardKinematics::getJacobianUnactuatedJoints()
{
    Eigen::MatrixXd J(jacobian_.rows(), non_actuated_joints_indexes_.size());
    for (std::size_t i = 0; i < non_actuated_joints_indexes_.size(); i++)
        J.col(i) = jacobian_.col(non_actuated_joints_indexes_[i]);

    return J;
}

