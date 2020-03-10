#ifndef GRAVITY_COMPENSATION_HELPERS_H
#define GRAVITY_COMPENSATION_HELPERS_H

// iDynTree includes
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>


namespace wholeBodyDynamics
{

/**
 * Class computing the gravity compensation torques
 * using the same accelerometers measurement used by
 * wholeBodyDynamics.
 *
 * NOTE : the estimation of the gravity disregards
 * as negligible the non-gravitational acceleration
 * measured by the IMU accelerometer.
 *
 */
class GravityCompensationHelper
{
private:
    iDynTree::Model m_model;
    bool m_isModelValid;
    bool m_isKinematicsUpdated;

    /**< Traveral used for the dynamics computations */
    iDynTree::Traversal m_dynamicTraversal;

    /**
     * Vector of Traversal used for the kinematic computations.
     * m_kinematicTraversals[l] contains the traversal with base link l .
     */
    std::vector<iDynTree::Traversal *> m_kinematicTraversals;

    /**
     * Helper functions for dealing with the kinematic traversal dynamic allocation
     */
    void allocKinematicTraversals(const size_t nrOfLinks);
    void freeKinematicTraversals();

    iDynTree::JointPosDoubleArray m_jointPos;
    iDynTree::JointDOFsDoubleArray m_jointDofsZero;
    iDynTree::LinkVelArray m_linkVels;
    iDynTree::LinkAccArray m_linkProperAccs;
    iDynTree::LinkNetExternalWrenches m_linkNetExternalWrenchesZero;
    iDynTree::LinkInternalWrenches    m_linkIntWrenches;
    iDynTree::FreeFloatingGeneralizedTorques m_generalizedTorques;

public:
    /**
     * Default constructor
     */
    GravityCompensationHelper();

    ~GravityCompensationHelper();

    /**
     * Load model
     */
    bool loadModel(const iDynTree::Model & _model , const std::string dynamicBase);

    /**
     * Set the kinematic information necessary for the gravity torques estimation using the
     * proper acceleration coming from an acceleromter.
     *
     * NOTE : the estimation of the gravity disregards
     * as negligible the non-gravitational acceleration
     * measured by the IMU accelerometer.
     *
     * @param[in] jointPos the position of the joints of the model.
     * @param[in] floatingFrame the index of the frame for which proper acceleration is provided.
     * @param[in] properClassicalLinearAcceleration proper (actual acceleration-gravity) classical acceleration
     *                                              of the origin of the specified frame,
     *                                              expressed in the specified frame orientation.
     * @return true if all went ok, false otherwise.
     */
    bool updateKinematicsFromProperAcceleration(const iDynTree::JointPosDoubleArray  & jointPos,
                                                const iDynTree::FrameIndex & floatingFrame,
                                                const iDynTree::Vector3 & properClassicalLinearAcceleration);

    /**
     * Set the kinematic information necessary for the gravity torques estimation using the
     * assumed known gravity vector on frame.
     *
     * \note This is implemented as updateKinematicsFromProperAcceleration(jointPos,floatingFrame,-gravity);
     *
     * @param[in] jointPos the position of the joints of the model.
     * @param[in] floatingFrame the index of the frame for which proper acceleration is provided.
     * @param[in] properClassicalLinearAcceleration gravity acceleration
     *                                              ot the origin of the specified frame,
     *                                              expressed in the specified frame orientation.
     * @return true if all went ok, false otherwise.
     */
    bool updateKinematicsFromGravity(const iDynTree::JointPosDoubleArray  & jointPos,
                                     const iDynTree::FrameIndex & floatingFrame,
                                     const iDynTree::Vector3 & gravity);


    /**
     * Get the gravity compensation torques.
     */
    bool getGravityCompensationTorques(iDynTree::JointDOFsDoubleArray & jointTrqs);

};

}

#endif
