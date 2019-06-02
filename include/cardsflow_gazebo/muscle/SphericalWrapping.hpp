#pragma once

#include "StateMachine.hpp"
#include "IViaPoints.hpp"

#include <boost/numeric/odeint.hpp>

namespace cardsflow_gazebo
{
	using namespace gazebo;

	class ITendon;

	class SphericalWrapping : public IViaPoints
	{

    public:
        SphericalWrapping();
        SphericalWrapping(gazebo::math::Vector3d point, physics::LinkPtr link);
        SphericalWrapping(gazebo::math::Vector3d point, double radius, int state, int counter, physics::LinkPtr link);

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
        gazebo::math::Vector3d prevCoord;
        gazebo::math::Vector3d nextCoord;
        gazebo::math::Vector3d normal;
        double arcAngle;
	};

    typedef boost::shared_ptr<SphericalWrapping> SphericalWrappingPtr;
}