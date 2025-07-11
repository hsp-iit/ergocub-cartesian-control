// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

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

#endif
