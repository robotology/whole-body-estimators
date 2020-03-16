#include "KnownExternalWrench.h"

namespace wholeBodyDynamics
{

KnownExternalWrench::KnownExternalWrench::KnownExternalWrench()
{
}

KnownExternalWrench::KnownExternalWrench(std::string parsedframeName, std::string parsedType, std::vector<double> parsedDirection, std::vector<double> parsedPosition)
{
    frameName = parsedframeName;
    type = parsedType;
    position = parsedPosition;
    direction = parsedDirection;
    
    if(type=="full")
    {
        noOfVariables = 6;
    }
    else if(type=="pure")
    {
        noOfVariables = 3;
    }
    else if(type=="pureKnown")
    {
        noOfVariables = 1;
    }
}

bool KnownExternalWrench::isValid(wholeBodyDynamics::KnownExternalWrench _knownExternalWrench)
{
    // For each frame in the paramater overrideContactFrames, there is one string value in the paramater contactWrenchType (one of the 3 types mentioned above), and 3 real numbers in the paramater contactWrenchDirection
    if(_knownExternalWrench.isTypeValid() && _knownExternalWrench.isPositionValid() && _knownExternalWrench.isDirectionValid())
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool KnownExternalWrench::isTypeValid()
{
    //Check is type is either:
    //full = FULL_WRENCH
    //pure = PURE_FORCE or
    //pureKnown = PURE_FORCE_WITH_KNOWN_DIRECTION
    if(type !="full" || type!="pure" || type !="pureKnown")
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool KnownExternalWrench::isPositionValid()
{
    //Make sure that the parsed postion vector is of 3 elements
    if(position.size() != 3)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool KnownExternalWrench::isDirectionValid()
{
    //Make sure that the parsed postion vector is of 3 elements
    if(direction.size() != 3)
    {
        return false;
    }
    else
    {
        return true;
    }
}

iDynTree::UnknownWrenchContactType KnownExternalWrench::asiDynTreeType()
{
    if(type=="full")
    {
        return iDynTree::FULL_WRENCH;
    }
    else if(type=="pure")
    {
        return iDynTree::PURE_FORCE;
    }
    else if(type=="pureKnown")
    {
        return iDynTree::PURE_FORCE_WITH_KNOWN_DIRECTION;
    }
}

iDynTree::Position KnownExternalWrench::asiDynTreePosition()
{
    return iDynTree::Position(position[0], position[1],position[3]);
}

iDynTree::Direction KnownExternalWrench::asiDynTreeDirection()
{
    if(type=="full" || type=="pure")
    {
        return iDynTree::Direction::Default();
    }
    else if(type=="pureKnown")
    {
        return iDynTree::Direction(direction[0], direction[1], direction[2]);
    }
}

void KnownExternalWrench::display()
{
    std::cout << "[KnownExternalWrench] Frame name: " << frameName << std::endl;
    std::cout << "                                                 Type: " << type << std::endl;
    std::cout << "                                                 position: " << position[0] << ", " << position[1] << ", " << position[2] << std::endl;
    std::cout << "                                                 position: " << direction[0] << ", " << direction[1] << ", " << direction[2] << std::endl;
}

}
