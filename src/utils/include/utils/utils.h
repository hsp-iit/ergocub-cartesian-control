#ifndef UTILS_H
#define UTILS_H

#include <yarp/os/ResourceFinder.h>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace utils
{
    enum class ParameterType {Bool, Int8, Int16, Int32, Int64, Float32, Float64, String};
    std::string parameterToString(const utils::ParameterType& type);
    Eigen::VectorXd loadVectorInt(const yarp::os::Bottle& rf, const std::string& key);
    Eigen::VectorXd loadVectorDouble(const yarp::os::Bottle& rf, const std::string& key);
    std::vector<std::string> loadVectorString(const yarp::os::Bottle& rf, const std::string& key);
    double evaluateReachabilityScore(const Eigen::Transform<double, 3, Eigen::Affine>& target_pose, const Eigen::Transform<double, 3, Eigen::Affine>& reachable_pose, const double position_threshold, const double orientation_threshold);
}


std::string utils::parameterToString(const utils::ParameterType& type);


/**
 * Given a yarp::os::Bottle, extract a vector of ints using the provided key.
 * This method assumes that the key matching, i.e. bottle.check(key) == true,
 * the existence of the vector, i.e. bottle.find(key).isList() == true,
 * and the validity of the type of each element of the list,
 * i.e. bottle.find(key)->get(i).isInt32() == true,
 * have been all already verified.
 * @param rf A yarp::os::Bottle bottle containing the list of interest.
 * @param key A std::string containing the name of the key used to access the list.
 * @return A Eigen::VectorXd containing the extracted vector.
 */
Eigen::VectorXd utils::loadVectorInt(const yarp::os::Bottle& bottle, const std::string& key);


/**
 * Given a yarp::os::Bottle, extract a vector of doubles using the provided key.
 * This method assumes that the key matching, i.e. bottle.check(key) == true,
 * the existence of the vector, i.e. bottle.find(key).isList() == true,
 * and the validity of the type of each element of the list,
 * i.e. bottle.find(key)->get(i).isFloat64() == true,
 * have been all already verified.
 * @param rf A yarp::os::Bottle bottle containing the list of interest.
 * @param key A std::string containing the name of the key used to access the list.
 * @return A Eigen::VectorXd containing the extracted vector.
 */
Eigen::VectorXd utils::loadVectorDouble(const yarp::os::Bottle& bottle, const std::string& key);


/**
 * Given a yarp::os::Bottle, extract a vector of std::string using the provided key.
 * This method assumes that the key matching, i.e. bottle.check(key) == true,
 * the existence of the vector, i.e. bottle.find(key).isList() == true,
 * and the validity of the type of each element of the list,
 * i.e. bottle.find(key)->get(i).isString() == true,
 * have been all already verified.
 * @param rf A yarp::os::Bottle bottle containing the list of interest.
 * @param key A std::string containing the name of the key used to access the list.
 * @return A std::vector<std::string> containing the extracted vector.
 */
std::vector<std::string> utils::loadVectorString(const yarp::os::Bottle& bottle, const std::string& key);

/**
 * Given a set of target and reachable end-effector poses, it returns a score consisting in
 * the rotational error between the two poses in the range [0, 1], if the positional error and the orientation error is below a given threshold.
 * Otherwise, the score gets assigned a maximu value of std::numeric_limits<double>::infinity().
 * @param target_pose A Eigen::Transform<double, 3, Eigen::Affine> containing the target pose of the end-effector.
 * @param reachable_pose A Eigen::Transform<double, 3, Eigen::Affine> containing the reachable pose of the end-effector.
 * @return The score as a double.
 */
double utils::evaluateReachabilityScore(const Eigen::Transform<double, 3, Eigen::Affine>& target_pose, const Eigen::Transform<double, 3, Eigen::Affine>& reachable_pose, const double position_threshold, const double orientation_threshold);

#endif
