#ifndef KNOWNEXTERNALWRENCH_H
#define KNOWNEXTERNALWRENCH_H

#include <array>
#include <vector>

#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>
//#include <iDynTree/Core/Position.h>
//#include <iDynTree/Core/Direction.h>

namespace wholeBodyDynamics
{

/**
 * Class for processing the raw measure of
 * a Six Axis Force Torque sensor.
 *
 * In particular this class process the raw measurement of a
 * Six Axis FT sensor using an affine function:
 *
 * ftNew = secondaryCalibrationMatrix*ftOld - offset
 *
 * Where offset is the offset of the F/T sensor (tipically computed
 * online at given instants where only one external force is known
 * to be acting on the robot, while secondaryCalibrationMatrix is a
 * secondary calibration matrix, that is tipically set to the identity.
 *
 */
class KnownExternalWrench
{
private:
    std::string frameName;
    std::string type;
    std::vector<double> position;
    std::vector<double> direction;

public:
    uint8_t noOfVariables;

    /**
     * Default constructor: the secondaryCalibrationMatrix is
     * set to the identity, while the offset is set to 0.
     */
    KnownExternalWrench();

    /**
     *  Copy constructor
     * TODO
     */
    KnownExternalWrench(std::string parsedframeName, std::string parsedType, std::vector<double> parsedPosition, std::vector<double> parsedDirection);

    /**
     * Process the input F/T by only applyng the calibration matrix.
     */
    bool isValid(KnownExternalWrench);

    /**
     * Process the input F/T by only applyng the calibration matrix.
     */
    bool isTypeValid();

    /**
     * Process the input F/T by only applyng the calibration matrix.
     */
    bool isPositionValid();

    /**
     * Process the input F/T by only applyng the calibration matrix.
     */
    bool isDirectionValid();

    /**
     * Process the input F/T by only applyng the calibration matrix.
     */
    iDynTree::UnknownWrenchContactType asiDynTreeType();

    /**
     * Process the input F/T by only applyng the calibration matrix.
     */
    iDynTree::Position asiDynTreePosition();

    /**
     * Process the input F/T by only applyng the calibration matrix.
     */
    iDynTree::Direction asiDynTreeDirection();
    
    /**
     * Print the contents of an instance in the console.
     */
    void display();
};
}


#endif // KNOWNEXTERNALWRENCH_H
