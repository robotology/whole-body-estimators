/**
 * @file Utils.tpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <iostream>

// YARP
#include <yarp/os/LogStream.h>

template <unsigned int n>
bool YarpHelper::yarpListToiDynTreeVectorFixSize(const yarp::os::Value& input, iDynTree::VectorFixSize<n>& output)
{
    if (input.isNull())
    {
        yError() << "[yarpListToiDynTreeVectorFixSize] Empty input value.";
        return false;
    }
    if (!input.isList() || !input.asList())
    {
        yError() << "[yarpListToiDynTreeVectorFixSize] Unable to read the input list.";
        return false;
    }
    yarp::os::Bottle *inputPtr = input.asList();

    if (inputPtr->size() != n)
    {
        yError() << "[yarpListToiDynTreeVectorFixSize] The dimension set in the configuration file is not "
                 << n;
        return false;
    }

    for (int i = 0; i < inputPtr->size(); i++)
    {
        if (!inputPtr->get(i).isFloat64() && !inputPtr->get(i).isInt32())
        {
            yError() << "[yarpListToiDynTreeVectorFixSize] The input is expected to be a double";
            return false;
        }
        output(i) = inputPtr->get(i).asFloat64();
    }
    return true;
}

template <typename T>
void YarpHelper::mergeSigVector(yarp::sig::Vector& vector, const T& t)
{
    for(int i= 0; i<t.size(); i++)
        vector.push_back(t(i));

    return;
}

template <typename T, typename... Args>
void YarpHelper::mergeSigVector(yarp::sig::Vector& vector, const T& t, const Args&... args)
{
    for(int i= 0; i<t.size(); i++)
        vector.push_back(t(i));

    mergeSigVector(vector, args...);

    return;
}

template <typename... Args>
void YarpHelper::sendVariadicVector(yarp::os::BufferedPort<yarp::sig::Vector>& port, const Args&... args)
{
    yarp::sig::Vector& vector = port.prepare();
    vector.clear();

    mergeSigVector(vector, args...);

    port.write();
}

template<typename T>
bool StdHelper::appendVectorToDeque(const std::vector<T>& input, std::deque<T>& output, const size_t& initPoint)
{
    if(initPoint > output.size())
    {
        std::cerr << "[appendVectorToDeque] The init point has to be less or equal to the size of the output deque."
                  << std::endl;
        return false;
    }

    // resize the deque
    output.resize(input.size() + initPoint);

    // Advances the iterator it by initPoint positions
    typename std::deque<T>::iterator it = output.begin();
    std::advance(it, initPoint);

    // copy the vector into the deque from the initPoint position
    std::copy(input.begin(), input.end(), it);

    return true;
}
