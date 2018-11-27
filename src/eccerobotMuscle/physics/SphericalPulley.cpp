/*
 *  Copyright (c) 2012-2013, MYOROBOTICS consortium
 *  Author: Steffen Wittmeier, Konstantinos Dalamagkidis, Michael Jaentsch
 *  All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification is governed by the MYOROBOTICS Non-Commercial Software
 *  License Agreement. See LICENSE file distributed with this work for
 *  additional information.
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under this license is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either expressed or
 *  implied. See the License for the specific language governing permissions
 *  and limitations under the License.
 *
 */

#include <rl/math/Rotation.h>
#include <rl/util/Timer.h>
#include "SphericalPulley.h"

namespace de
{
namespace caliper
{
namespace sim
{
namespace ext
{
namespace actuators
{
namespace er
{
namespace physics
{
SphericalPulley::SphericalPulley(sgal::physics::Body* body,
		const ::rl::math::Transform &frame, IMuscle* muscle,
		::rl::math::Real radius, PulleyStateMachine::State initialState,
		unsigned int numHalfRevolutions) :
		AttachmentPoint(body, frame, muscle), pulleyStateMachine(initialState,
				numHalfRevolutions), radius(radius), arcAngle(0.0)
{
}

SphericalPulley::~SphericalPulley()
{

}

IAttachmentPoint::Type SphericalPulley::getType() const
{
	return IAttachmentPoint::SphericalPulley;
}

void SphericalPulley::updateForcePoints(rl::math::Real time)
		throw (sgal::Exception)
{
	previousAttachmentPointOrigin = frameGlobal.rotation().transpose()
			* (previous->getPreviousForcePointInGlobal()
					- frameGlobal.translation());

	nextAttachmentPointOrigin = frameGlobal.rotation().transpose()
			* (next->getNextForcePointInGlobal() - frameGlobal.translation());

	// check whether an anchor penetrates the sphere
	if (previousAttachmentPointOrigin.norm()
			< radius * (1 + std::numeric_limits< double >::epsilon())
			|| nextAttachmentPointOrigin.norm()
					< radius * (1 + std::numeric_limits< double >::epsilon()))
	{
		QString string = "Spherical pulley anchor of muscle "
				+ QString(muscle->getName().c_str())
				+ " penetrates pulley sphere.";

		throw sgal::Exception(string.toStdString().c_str());
	}

	N = previousAttachmentPointOrigin.cross(nextAttachmentPointOrigin);

	N.normalize();

	pulleyStateMachine.updateState(previousAttachmentPointOrigin,
			nextAttachmentPointOrigin, N, radius);

	switch (pulleyStateMachine.getState())
	{
	case PulleyStateMachine::NotWrapping:
	{
		previousForcePoint = nextAttachmentPointOrigin;

		nextForcePoint = previousAttachmentPointOrigin;

		tangentPointA1 = tangentPointA2 = tangentPointB1 = tangentPointB2 =
				rl::math::Vector3::Zero();

		break;
	}
	case PulleyStateMachine::Positive:
	{
		rl::math::Vector3 tangentPointA, tangentPointB;

		PulleyStateMachine::computeTangentPoint(previousAttachmentPointOrigin,
				N, radius, tangentPointA, tangentPointB);

		tangentPointA1 = previousForcePoint = tangentPointA;

		tangentPointA2 = tangentPointB;

		PulleyStateMachine::computeTangentPoint(nextAttachmentPointOrigin, N,
				radius, tangentPointA, tangentPointB);

		tangentPointB1 = nextForcePoint = tangentPointB;

		tangentPointB2 = tangentPointA;

		break;
	}
	case PulleyStateMachine::Negative:
	{
		rl::math::Vector3 tangentPointA, tangentPointB;

		PulleyStateMachine::computeTangentPoint(previousAttachmentPointOrigin,
				N, radius, tangentPointA, tangentPointB);

		tangentPointA1 = tangentPointA;

		tangentPointA2 = previousForcePoint = tangentPointB;

		PulleyStateMachine::computeTangentPoint(nextAttachmentPointOrigin, N,
				radius, tangentPointA, tangentPointB);

		tangentPointB1 = tangentPointB;

		tangentPointB2 = nextForcePoint = tangentPointA;

		break;
	}
	}

	pulleyStateMachine.updateRevolutionCounter(previousForcePoint,
			nextForcePoint);

	arcAngle = pulleyStateMachine.getArcAngle(previousForcePoint,
			nextForcePoint, radius);

	previousForcePointInGlobal = frameGlobal.translation()
			+ frameGlobal.rotation() * previousForcePoint;

	nextForcePointInGlobal = frameGlobal.translation()
			+ frameGlobal.rotation() * nextForcePoint;
}

void SphericalPulley::applyForce(rl::math::Real time,
		rl::math::Real previousSegmentKiteLineVelocity, bool beforeSEE)
				throw (sgal::Exception)
{
	// no friction in pulley but required to correctly propagate forces through tendon path
	if (beforeSEE)
	{
		fa = fb = next->getPreviousForce();
	}
	else
	{
		fb = fa = previous->getNextForce();
	}

	if (pulleyStateMachine.getState() != PulleyStateMachine::NotWrapping)
	{
		// vector from previous to tangentPointA
		A = previous->getNextForcePointInGlobal()
				- getPreviousForcePointInGlobal();

		Fa = A.normalized() * fa;

		// vector from tangentPointB to next
		B = next->getPreviousForcePointInGlobal() - getNextForcePointInGlobal();

		Fb = B.normalized() * fb;

		body->applyForce(Fa,
				getPreviousForcePointInGlobal() - bodyFrame.translation());

		body->applyForce(Fb,
				getNextForcePointInGlobal() - bodyFrame.translation());
	}
}

rl::math::Vector3 SphericalPulley::getPreviousForcePointInGlobal() const
{
	return previousForcePointInGlobal;
}

rl::math::Vector3 SphericalPulley::getNextForcePointInGlobal() const
{
	return nextForcePointInGlobal;
}

rl::math::Real SphericalPulley::getPreviousSegmentLength() const
{
	if (pulleyStateMachine.getState() != PulleyStateMachine::NotWrapping)
	{
		return (previous->getNextForcePointInGlobal()
				- getPreviousForcePointInGlobal()).norm() + arcAngle * radius;
	}

	return 0;
}

ISphericalPulley::State SphericalPulley::getState() const
{
	if (pulleyStateMachine.getState() == PulleyStateMachine::NotWrapping)
	{
		return ISphericalPulley::NotWrapping;
	}
	else
	{
		return ISphericalPulley::Wrapping;
	}
}

const rl::math::Vector3& SphericalPulley::getN() const
{
	return N;
}

const rl::math::Vector3& SphericalPulley::getTangentPointA1() const
{
	return tangentPointA1;
}

const rl::math::Vector3& SphericalPulley::getTangentPointB1() const
{
	return tangentPointB1;
}

const rl::math::Vector3& SphericalPulley::getTangentPointA2() const
{
	return tangentPointA2;
}

const rl::math::Vector3& SphericalPulley::getTangentPointB2() const
{
	return tangentPointB2;
}

rl::math::Real SphericalPulley::getRadius() const
{
	return radius;
}

void SphericalPulley::setRadius(rl::math::Real radius)
{
	this->radius = radius;
}

void SphericalPulley::getTendonNodes(QVector< rl::math::Vector3 > &nodes)
{
	// only compute arc nodes when wrapping
	if (pulleyStateMachine.getState() != PulleyStateMachine::NotWrapping)
	{
		// compute the sign of the rotation
		// (the arc nodes are always computed by rotating
		// the previous force point into the next force point
		double sign = 1.0;

		if (pulleyStateMachine.getState() == PulleyStateMachine::Negative)
		{
			sign = -1.0;
		}

		// compute the number of arc nodes and the arc step
		int arcNodes = std::ceil(arcAngle / 0.1);

		double arcStep = arcAngle / arcNodes;

		rl::math::Vector3 node;

		for (int i = 0; i <= arcNodes; i++)
		{
			rl::math::AngleAxis angleAxis(sign * i * arcStep, N);

			node = angleAxis.toRotationMatrix() * previousForcePoint;

			nodes.push_back(
					frameGlobal.translation() + frameGlobal.rotation() * node);
		}
	}
}
}
}
}
}
}
}
}
