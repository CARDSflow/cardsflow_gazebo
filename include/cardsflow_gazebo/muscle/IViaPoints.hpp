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

        IViaPoints(math::Vector3 point);

        IViaPoints(math::Vector3 point, physics::LinkPtr link);

        IViaPoints(math::Vector3 point, Type type, physics::LinkPtr link);

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
        math::Vector3 linkPosition; //global
        math::Quaternion linkRotation; //global
        math::Vector3 localCoordinates;
        math::Vector3 globalCoordinates;
        Type type;
        IViaPointsPtr prevPoint;
        IViaPointsPtr nextPoint;
        math::Vector3 prevForcePoint; //global
        math::Vector3 nextForcePoint; //global
        double fa;
        double fb;
        math::Vector3 prevForce;
        math::Vector3 nextForce;
        double previousSegmentLength;
	};

	struct ViaPointInfo{
		math::Vector3 local_coordinates;
		math::Vector3 global_coordinates;
		IViaPoints::Type type;
		double radius;
		int state;
		int revCounter;
		physics::LinkPtr link;
		std::string link_name;
	};
}