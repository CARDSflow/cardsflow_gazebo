#pragma once

#include "StateMachine.hpp"
#include <ros/ros.h>

namespace cardsflow_gazebo
{
	using namespace gazebo;

    class IViaPoints;
    typedef boost::shared_ptr<IViaPoints> IViaPointsPtr;

	class IViaPoints
	{

    public:
        ////////////////////
        /// \brief The type enum for different attachment point types.
        ///
        /// Differentiates between four possible attachment point types:
        /// 0: Fix point
        /// 1: Spherical Wrapping Surface
        /// 2: Cylindrical Wrapping Surface
        /// 3: Mesh Wrapping Surface
        enum Type
        {
            FIXPOINT = 0,
            SPHERICAL = 1,
            CYLINDRICAL = 2,
            MESH = 3
        };

        IViaPoints();

        IViaPoints(gazebo::math::Vector3d point);

        IViaPoints(gazebo::math::Vector3d point, physics::LinkPtr link);

        IViaPoints(gazebo::math::Vector3d point, Type type, physics::LinkPtr link);

        ////////////////////
        /// \brief This function updates the position of the attachment point.
        ///
        /// Retrives the links position from Gazebo.
        virtual void UpdateForcePoints();

        ////////////////////
        /// \brief This function applies the muscle force to the attachment point
        virtual void CalculateForce();

    public:
        physics::LinkPtr link;
        physics::JointPtr joint = nullptr;
        gazebo::math::Vector3d linkPosition; //global
        gazebo::math::Quaterniond linkRotation; //global
        gazebo::math::Vector3d localCoordinates;
        gazebo::math::Vector3d globalCoordinates;
        Type type;
        IViaPointsPtr prevPoint;
        IViaPointsPtr nextPoint;
        gazebo::math::Vector3d prevForcePoint; //global
        gazebo::math::Vector3d nextForcePoint; //global
        double fa;
        double fb;
        gazebo::math::Vector3d prevForce;
        gazebo::math::Vector3d nextForce;
        double previousSegmentLength;
	};

	struct ViaPointInfo{
		gazebo::math::Vector3d local_coordinates;
		gazebo::math::Vector3d global_coordinates;
		IViaPoints::Type type;
		double radius;
		int state;
		int revCounter;
		physics::LinkPtr link;
		std::string link_name;
	};
}