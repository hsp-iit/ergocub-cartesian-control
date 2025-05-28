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
