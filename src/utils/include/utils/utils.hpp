// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef UTILS_HPP
#define UTILS_HPP

#include <utils/utils.h>

#include <yarp/os/LogStream.h>

namespace utils
{
    template<typename T>
    bool checkParameters(const std::vector<std::vector<std::string>>& parameters_vector, const std::string& separator, const T& resource_storage, const std::string& group, const utils::ParameterType& type, const bool& is_list = false);
}


/**
 * Given a vector of vectors v_i of strings, check if the parameters obtained from
 * all the possible ordered combinations of string keys from the vectors v_i, joined with a seperator,
 * are provided within the input resource storage and are of the specified type.
 *
 * @param parameters_vector A std::vector<std::vector<std::string>> vector of vectors of strings containing the parameters keys.
 * @param separator A std::string containing the separator to be used to join the keys.
 * @param resource A yarp::os::ResourceFinder resource finder.
 * @param group A std::string indicating the name of the group of the resource finder to search within.
 * @param type A utils::ParameterType indicating the parameters type among 'int{8, 16, 32, 64}', 'float{32, 64}', 'string'.
 * @param is_list A boolean indicating whether parameters are list of elements of type 'type' - false by default.
 * @return true if all the parameters are provided, false otherwise.
 */
template<typename T>
bool utils::checkParameters
(
    const std::vector<std::vector<std::string>>& parameters_vector,
    const std::string& separator,
    const T& resource,
    const std::string& group,
    const utils::ParameterType& type,
    const bool& is_list
)
{
    const std::string parameter_type_str = utils::parameterToString(type);

    /* Compose all possible combinations of strings from each vector of strings. */
    std::vector<std::string> params;
    if (parameters_vector.size() == 1)
        params = parameters_vector[0];
    else
    {
        std::vector<std::string> params_prev = parameters_vector.at(0);
        for (std::size_t i = 1; i < parameters_vector.size(); i++)
        {
            for (const std::string& key_prev : params_prev)
                for (const std::string& key_next : parameters_vector.at(i))
                    params.push_back(key_prev + separator + key_next);
            params_prev = params;

            if (i < parameters_vector.size() - 1)
                params.clear();
        }
    }

    /* Validity checker helper. */
    const auto is_type_valid = [](const yarp::os::Value& value, const utils::ParameterType& type)
    {
        return (type == utils::ParameterType::Bool && value.isBool())       ||
               (type == utils::ParameterType::Int8 && value.isInt8())       ||
               (type == utils::ParameterType::Int16 && value.isInt16())     ||
               (type == utils::ParameterType::Int32 && value.isInt32())     ||
               (type == utils::ParameterType::Int64 && value.isInt64())     ||
               (type == utils::ParameterType::Float32 && value.isFloat32()) ||
               (type == utils::ParameterType::Float64 && value.isFloat64()) ||
               (type == utils::ParameterType::String  && value.isString());
    };

    /* Check parameters. */
    bool valid_all = true;
    const yarp::os::Bottle b = resource.findGroup(group);
    std::string error_string_prefix = group.empty() ? "" : group + "::";
    for (const std::string& name : params)
    {
        /* The parameter does not exist. */
        if ((!group.empty() && !b.check(name)) || (group.empty() && !resource.check(name)))
        {
            valid_all = false;
            yError() << "utils::checkParameters(). Error: cannot find parameter " + error_string_prefix + name + ".";
        }
        /* Check parameter type. */
        else
        {
            const auto value = group.empty() ? resource.find(name) : b.find(name);
            /* Handle a list if required. */
            if (is_list)
            {
                if (!value.isList())
                {
                    yError() << "utils::checkParameters(). Error: parameter " + error_string_prefix + name + " should be of a list of type " + parameter_type_str + ".";
                    valid_all = false;
                }
                else
                {
                    const auto list = value.asList();
                    for (std::size_t i = 0; i < list->size(); i++)
                        if(!is_type_valid(list->get(i), type))
                        {
                            yError() << "utils::checkParameters(). Error: parameter " + error_string_prefix + name + " should be of a list of type " + parameter_type_str + ".";
                            valid_all = false;
                            break;
                        }
                }

            }
            /* Otherwise consider the value as a single item. */
            else if (!is_type_valid(value, type))
            {
                valid_all = false;
                yError() << "utils::checkParameters(). Error: parameter " + error_string_prefix + name + " should be of type " + parameter_type_str + ".";
            }
        }
    }
    return valid_all;
}

#endif
