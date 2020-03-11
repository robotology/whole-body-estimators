#include "SixAxisForceTorqueMeasureHelpers.h"
#include <iDynTree/Core/EigenHelpers.h>


namespace wholeBodyDynamics
{

SixAxisForceTorqueMeasureProcessor::SixAxisForceTorqueMeasureProcessor()
{
    // Initialize the affice function to be the identity
    toEigen(m_secondaryCalibrationMatrix).setIdentity();
    m_offset.zero();
    toEigen(m_temperatureCoefficients).setZero();
    m_tempOffset=0.0;
    m_estimated_offset.zero();
}

iDynTree::Matrix6x6& SixAxisForceTorqueMeasureProcessor::secondaryCalibrationMatrix()
{
    return m_secondaryCalibrationMatrix;
}

iDynTree::Vector6& SixAxisForceTorqueMeasureProcessor::temperatureCoefficients()
{
    return m_temperatureCoefficients;
}


const iDynTree::Matrix6x6& SixAxisForceTorqueMeasureProcessor::secondaryCalibrationMatrix() const
{
    return m_secondaryCalibrationMatrix;
}

const iDynTree::Vector6& SixAxisForceTorqueMeasureProcessor::temperatureCoefficients() const
{
    return m_temperatureCoefficients;
}

iDynTree::Wrench& SixAxisForceTorqueMeasureProcessor::offset()
{
    return m_offset;
}

const iDynTree::Wrench& SixAxisForceTorqueMeasureProcessor::offset() const
{
    return m_offset;
}

iDynTree::Wrench& SixAxisForceTorqueMeasureProcessor::estimatedOffset()
{
    return m_estimated_offset;
}

const iDynTree::Wrench& SixAxisForceTorqueMeasureProcessor::estimatedOffset() const
{
    return m_estimated_offset;
}

double& SixAxisForceTorqueMeasureProcessor::tempOffset()
{
    return m_tempOffset;
}

const double& SixAxisForceTorqueMeasureProcessor::tempOffset() const
{
    return m_tempOffset;
}

iDynTree::Wrench SixAxisForceTorqueMeasureProcessor::filt(const iDynTree::Wrench& input) const
{
    Eigen::Matrix<double,6,1> retEig = toEigen(m_secondaryCalibrationMatrix)*toEigen(input)-toEigen(m_offset);
    iDynTree::Wrench ret;
    fromEigen(ret,retEig);
    return ret;
}

iDynTree::Wrench SixAxisForceTorqueMeasureProcessor::filt(const iDynTree::Wrench & input, const double & temperature) const
{
    Eigen::Matrix<double,6,1> retEig = toEigen(m_secondaryCalibrationMatrix)*toEigen(input)-toEigen(m_offset)+toEigen(m_temperatureCoefficients)*(temperature-m_tempOffset);    
    iDynTree::Wrench ret;
    fromEigen(ret,retEig);


    return ret;
}

iDynTree::Wrench SixAxisForceTorqueMeasureProcessor::applySecondaryCalibrationMatrix(const iDynTree::Wrench& input) const
{
    Eigen::Matrix<double,6,1> retEig = toEigen(m_secondaryCalibrationMatrix)*toEigen(input);

    iDynTree::Wrench ret;
    fromEigen(ret,retEig);

    return ret;
}

iDynTree::Wrench SixAxisForceTorqueMeasureProcessor::applySecondaryCalibrationMatrix(const iDynTree::Wrench& input, const double & temperature) const
{
    Eigen::Matrix<double,6,1> retEig = toEigen(m_secondaryCalibrationMatrix)*toEigen(input)+toEigen(m_temperatureCoefficients)*(temperature-m_tempOffset);

    iDynTree::Wrench ret;
    fromEigen(ret,retEig);

    return ret;
}

}


