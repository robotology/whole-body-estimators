#ifndef SIX_AXIS_FORCE_TORQUE_MEASURE_PROCESSOR_H
#define SIX_AXIS_FORCE_TORQUE_MEASURE_PROCESSOR_H

// iDynTree includes
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/MatrixFixSize.h>


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
class SixAxisForceTorqueMeasureProcessor
{
private:
    iDynTree::Matrix6x6 m_secondaryCalibrationMatrix;
    iDynTree::Wrench m_offset;

public:
    /**
     * Default constructor: the secondaryCalibrationMatrix is
     * set to the identity, while the offset is set to 0.
     */
    SixAxisForceTorqueMeasureProcessor();

    /**
     * Accessor to the offset.
     */
    iDynTree::Wrench & offset();

    /**
     * Const accessor to the offset.
     */
    const iDynTree::Wrench & offset() const;

    /**
     * Accessor to the secondary calibration matrix.
     */
    iDynTree::Matrix6x6 & secondaryCalibrationMatrix();

    /**
     * Const accessor to the secondary calibration matrix.
     */
    const iDynTree::Matrix6x6 & secondaryCalibrationMatrix() const;

    /**
     * Process the input F/T.
     *
     * Returns secondaryCalibrationMatrix*input + offset .
     */
    iDynTree::Wrench filt(const iDynTree::Wrench & input) const;

    /**
     * Process the input F/T by only applyng the calibration matrix.
     */
    iDynTree::Wrench applySecondaryCalibrationMatrix(const iDynTree::Wrench & input) const;
};

}

#endif
