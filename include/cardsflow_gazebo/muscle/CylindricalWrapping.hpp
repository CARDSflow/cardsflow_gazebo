#pragma once

#include "StateMachine.hpp"
#include "IViaPoints.hpp"

#include <boost/numeric/odeint.hpp>

namespace cardsflow_gazebo
{
	using namespace gazebo;

	class ITendon;

	class CylindricalWrapping : public IViaPoints
	{

    public:
        CylindricalWrapping();
        CylindricalWrapping(ignition::math::Vector3d point, physics::LinkPtr link);
        CylindricalWrapping(ignition::math::Vector3d point, double radius, int state, int counter, physics::LinkPtr link);

        ////////////////////
        /// \brief This function updates the position of the attachment point.
        ///
        /// Retrives the links position from Gazebo.
        virtual void UpdateForcePoints();

        ////////////////////
        /// \brief This function applies the muscle force to the attachment point
        virtual void CalculateForce();

    public:
        StateMachine stateMachine;
        double radius;
        ignition::math::Vector3d prevCoord;
        ignition::math::Vector3d nextCoord;
        ignition::math::Vector3d prevCoordPlane;
        ignition::math::Vector3d nextCoordPlane;
        ignition::math::Vector3d prevForcePointPlane;
        ignition::math::Vector3d nextForcePointPlane;
        ignition::math::Vector3d normal;
        double arcAngle;

	};

    typedef boost::shared_ptr<CylindricalWrapping> CylindricalWrappingPtr;
}