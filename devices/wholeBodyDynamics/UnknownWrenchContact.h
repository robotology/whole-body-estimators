#ifndef UNKNOWNWRENCHCONTACT_H
#define UNKNOWNWRENCHCONTACT_H

#include <vector>

#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>

namespace wholeBodyDynamics
{

/**
 * Class for storing the parsed information about an unknown 
 * external wrench to be estimated. This class inherits from the
 * structure 'iDynTree::UnknownWrenchContact' and contains 
 * in addition other information such as the frame name and
 * the number of the unknowns.
 */
class UnknownWrenchContact : public iDynTree::UnknownWrenchContact
{
private:
    std::string frameName;

public:
    uint8_t nrUnknownsInExtWrench;

    /**
     * Default constructor.
     */
    UnknownWrenchContact();

    /**
     *  Parameterized constructor: takes as inputs the information from a configuration file.
     */
    UnknownWrenchContact(std::string parsedframeName, std::string parsedType, std::vector<double> parsedDirection, std::vector<double> parsedPosition);

    /**
     * Check if the contact type is a valid
     * iDynTree::UnknownWrenchContactType object.
     */
    bool isTypeValid();

    /**
     * Print the frame name, type, position and direction in the console.
     */
    void display();
};
}


#endif // UNKNOWNWRENCHCONTACT_H
