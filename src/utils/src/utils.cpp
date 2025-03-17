#include <utils/utils.h>

#include <yarp/os/Bottle.h>


std::string utils::parameterToString(const utils::ParameterType& type)
{
    if (type == utils::ParameterType::Bool)
        return "Bool";
    else if (type == utils::ParameterType::Int8)
        return "Int8";
    else if (type == utils::ParameterType::Int16)
        return "Int16";
    else if (type == utils::ParameterType::Int32)
        return "Int32";
    else if (type == utils::ParameterType::Int64)
        return "Int64";
    else if (type == utils::ParameterType::Float32)
        return "Float32";
    else if (type == utils::ParameterType::Float64)
        return "Float64";
    else if (type == utils::ParameterType::String)
        return "String";

    return "not_valid_type";
}


Eigen::VectorXd utils::loadVectorInt(const yarp::os::Bottle& bottle, const std::string& key)
{
    const auto vector_yarp = bottle.find(key).asList();
    Eigen::VectorXd vector(vector_yarp->size());
    for (std::size_t i = 0; i < vector_yarp->size(); i++)
        vector(i) = vector_yarp->get(i).asInt32();

    return vector;
}


Eigen::VectorXd utils::loadVectorDouble(const yarp::os::Bottle& bottle, const std::string& key)
{
    const auto vector_yarp = bottle.find(key).asList();
    Eigen::VectorXd vector(vector_yarp->size());
    for (std::size_t i = 0; i < vector_yarp->size(); i++)
        vector(i) = vector_yarp->get(i).asFloat64();

    return vector;
}


std::vector<std::string> utils::loadVectorString(const yarp::os::Bottle& bottle, const std::string& key)
{
    const auto vector_yarp = bottle.find(key).asList();
    std::vector<std::string> vector;
    for (std::size_t i = 0; i < vector_yarp->size(); i++)
        vector.push_back(vector_yarp->get(i).asString());

    return vector;
}


double utils::evaluateReachabilityScore(const Eigen::Transform<double, 3, Eigen::Affine>& target_pose, const Eigen::Transform<double, 3, Eigen::Affine>& reachable_pose, const double position_threshold, const double orientation_threshold)
{
    /*  Evaluate the rotational error as the norm of the "vee" of the logarithm of the error matrix between target_pose.rotation and reacheable_pose.rotation.
        Such a metric provides values in [0, pi]
        References: https://arxiv.org/pdf/1812.01537.pdf, Fig. 6 (for the "vee" operator)
                    https://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf, Eq. 23 (for the rotational error definition)
        */
    Eigen::Matrix3d error = target_pose.rotation() * reachable_pose.rotation().transpose();
    Eigen::Vector3d vee_of_log = Eigen::AngleAxisd(error).axis() * Eigen::AngleAxisd(error).angle();
    double rotational_error = vee_of_log.norm();

    if ((target_pose.translation() - reachable_pose.translation()).norm() < position_threshold && rotational_error < orientation_threshold)
    {
        /* Divide by pi to obtain the score in [0, 1]*/
        return rotational_error / M_PI;
    }
    else
    {
        return std::numeric_limits<double>::infinity();
    }
}
