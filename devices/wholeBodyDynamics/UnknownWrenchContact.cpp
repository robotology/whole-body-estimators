#include "UnknownWrenchContact.h"

namespace wholeBodyDynamics
{

UnknownWrenchContact::UnknownWrenchContact()
{
}

UnknownWrenchContact::UnknownWrenchContact(std::string parsedframeName, std::string parsedType, std::vector<double> parsedDirection, std::vector<double> parsedPosition)
{
    frameName = parsedframeName;
    contactPoint = iDynTree::Position(parsedPosition[0], parsedPosition[1], parsedPosition[2]);

    if(parsedType=="full")
    {
        unknownType = iDynTree::FULL_WRENCH;
        forceDirection = iDynTree::Direction::Default();
        nrUnknownsInExtWrench = 6;
    }
    else if(parsedType=="pure")
    {
        unknownType = iDynTree::PURE_FORCE;
        forceDirection = iDynTree::Direction::Default();
        nrUnknownsInExtWrench = 3;
    }
    else if(parsedType=="pureKnown")
    {
        unknownType = iDynTree::PURE_FORCE_WITH_KNOWN_DIRECTION;
        forceDirection = iDynTree::Direction(parsedDirection[0], parsedDirection[1], parsedDirection[2]);
        nrUnknownsInExtWrench = 1;
    }
    else
    {
        // concerning this class, it's considered an invalid type
        unknownType = iDynTree::NO_UNKNOWNS;
    }
}

bool UnknownWrenchContact::isTypeValid()
{
    //Check is type is either:
    //full = FULL_WRENCH
    //pure = PURE_FORCE or
    //pureKnown = PURE_FORCE_WITH_KNOWN_DIRECTION
    if(unknownType !=iDynTree::FULL_WRENCH || unknownType!=iDynTree::PURE_FORCE || unknownType !=iDynTree::PURE_FORCE_WITH_KNOWN_DIRECTION)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void UnknownWrenchContact::display()
{
    std::cout << "[KnownExternalWrench] Frame name: " << frameName << std::endl;
    std::cout << "                                                 Type: " << unknownType << std::endl;
    std::cout << "                                                 position: " << contactPoint[0] << ", " << contactPoint[1] << ", " << contactPoint[2] << std::endl;
    std::cout << "                                                 position: " << forceDirection[0] << ", " << forceDirection[1] << ", " << forceDirection[2] << std::endl;
}

}
