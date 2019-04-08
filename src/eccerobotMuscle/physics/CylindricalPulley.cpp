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
#include "CylindricalPulley.h"

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
CylindricalPulley::CylindricalPulley(sgal::physics::Body* body,
		const ::rl::math::Transform &frame, IMuscle* muscle,
		::rl::math::Real radius, rl::math::Real height,
		PulleyStateMachine::State initialState, unsigned int numHalfRevolutions) :
		AttachmentPoint(body, frame, muscle), pulleyStateMachine(initialState,
				numHalfRevolutions), radius(radius), height(height)
{
}

CylindricalPulley::~CylindricalPulley()
{

}

IAttachmentPoint::Type CylindricalPulley::getType() const
{
	return IAttachmentPoint::CylindricalPulley;
}

void CylindricalPulley::updateForcePoints(rl::math::Real time)
		throw (sgal::Exception)
{
	previousAttachmentPointOrigin = previousAttachmentOriginInPlane =
			frameGlobal.rotation().transpose()
					* (previous->getNextForcePointInGlobal()
							- frameGlobal.translation());

	previousAttachmentOriginInPlane(1) = 0.0;

	// check whether the previous anchor penetrates the cylinder

	if (std::abs(previousAttachmentPointOrigin(1)) < height / 2.0)
	{
		if (previousAttachmentOriginInPlane.norm() < radius)
		{
			QString string = "Cylindrical pulley anchor of muscle "
					+ QString(muscle->getName().c_str())
					+ " penetrates pulley cylinder.";

			throw sgal::Exception(string.toStdString().c_str());
		}
	}

	nextAttachmentPointOrigin = nextAttachmentOriginInPlane =
			frameGlobal.rotation().transpose()
					* (next->getPreviousForcePointInGlobal()
							- frameGlobal.translation());

	nextAttachmentOriginInPlane(1) = 0.0;

	// check whether the next anchor penetrates the cylinder
	if (std::abs(nextAttachmentPointOrigin(1)) < height / 2.0)
	{
		if (nextAttachmentOriginInPlane.norm() < radius)
		{
			QString string = "Cylindrical pulley anchor of muscle "
					+ QString(muscle->getName().c_str())
					+ " penetrates pulley cylinder.";

			throw sgal::Exception(string.toStdString().c_str());
		}
	}

	N = previousAttachmentOriginInPlane.cross(nextAttachmentOriginInPlane);

	N.normalize();

// UPDATE FORCE POINTS
	pulleyStateMachine.updateState(previousAttachmentOriginInPlane,
			nextAttachmentOriginInPlane, N, radius);

	switch (pulleyStateMachine.getState())
	{
	case PulleyStateMachine::NotWrapping:
	{
		// invert projection in not wrapping case
		// (!! previousForcePoint is nextAttachmentOrigin and vice versa)
		previousForcePointInPlane = previousForcePoint =
				nextAttachmentPointOrigin;

		nextForcePointInPlane = nextForcePoint = previousAttachmentPointOrigin;

		break;
	}
	case PulleyStateMachine::Positive:
	{
		rl::math::Vector3 tangentPointA, tangentPointB;

		// if attachment is higher than cylinder, use cylinder edge
		PulleyStateMachine::computeTangentPoint(previousAttachmentOriginInPlane,
				N, radius, tangentPointA, tangentPointB);

		previousForcePointInPlane = previousForcePoint = tangentPointA;

		PulleyStateMachine::computeTangentPoint(nextAttachmentOriginInPlane, N,
				radius, tangentPointA, tangentPointB);

		nextForcePointInPlane = nextForcePoint = tangentPointB;

		break;
	}
	case PulleyStateMachine::Negative:
	{
		rl::math::Vector3 tangentPointA, tangentPointB;

		PulleyStateMachine::computeTangentPoint(previousAttachmentOriginInPlane,
				N, radius, tangentPointA, tangentPointB);

		previousForcePointInPlane = previousForcePoint = tangentPointB;

		PulleyStateMachine::computeTangentPoint(nextAttachmentOriginInPlane, N,
				radius, tangentPointA, tangentPointB);

		nextForcePointInPlane = nextForcePoint = tangentPointA;

		break;
	}
	}

	pulleyStateMachine.updateRevolutionCounter(previousForcePointInPlane,
			nextForcePointInPlane);

	arcAngle = pulleyStateMachine.getArcAngle(previousForcePointInPlane,
			nextForcePointInPlane, radius);

	if (pulleyStateMachine.getState() != PulleyStateMachine::NotWrapping)
	{
		// project back
		previousForcePoint(1) = computeYCoordinate(0.0);

		nextForcePoint(1) = computeYCoordinate(arcAngle);
	}

	previousForcePointInGlobal = frameGlobal.translation()
			+ frameGlobal.rotation() * previousForcePoint;

	nextForcePointInGlobal = frameGlobal.translation()
			+ frameGlobal.rotation() * nextForcePoint;
}

void CylindricalPulley::applyForce(rl::math::Real time,
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

void CylindricalPulley::getTendonNodes(QVector< rl::math::Vector3 > &nodes)
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

			node = angleAxis.toRotationMatrix() * previousForcePointInPlane;

			node(1) = computeYCoordinate(i * arcStep);

			nodes.push_back(
					frameGlobal.translation() + frameGlobal.rotation() * node);
		}
	}
}

rl::math::Real CylindricalPulley::getPreviousSegmentLength() const
{
	if (pulleyStateMachine.getState() != PulleyStateMachine::NotWrapping)
	{
		return (previous->getNextForcePointInGlobal()
				- getPreviousForcePointInGlobal()).norm()
				+ std::sqrt(
						std::pow(arcAngle * radius, 2)
								+ std::pow(
										previousForcePoint(1)
												- nextForcePoint(1), 2));
	}

	return 0;
}

rl::math::Vector3 CylindricalPulley::getPreviousForcePointInGlobal() const
{
	return previousForcePointInGlobal;
}

rl::math::Vector3 CylindricalPulley::getNextForcePointInGlobal() const
{
	return nextForcePointInGlobal;
}

ICylindricalPulley::State CylindricalPulley::getState() const
{
	if (pulleyStateMachine.getState() == PulleyStateMachine::NotWrapping)
	{
		return ICylindricalPulley::NotWrapping;
	}
	else
	{
		return ICylindricalPulley::Wrapping;
	}
}

rl::math::Real CylindricalPulley::getRadius() const
{
	return radius;
}

void CylindricalPulley::setRadius(rl::math::Real radius)
{
	this->radius = radius;
}

rl::math::Real CylindricalPulley::getHeight() const
{
	return height;
}

void CylindricalPulley::setHeight(rl::math::Real height)
{
	this->height = height;
}

const rl::math::Vector3& CylindricalPulley::getPreviousInPlane() const
{
	return previousAttachmentOriginInPlane;
}

const rl::math::Vector3& CylindricalPulley::getNextInPlane() const
{
	return nextAttachmentOriginInPlane;
}

const rl::math::Vector3& CylindricalPulley::getN() const
{
	return N;
}

double CylindricalPulley::computeYCoordinate(double angle) const
{
	double totalLength =
			radius * arcAngle
					+ (previousForcePointInPlane
							- previousAttachmentOriginInPlane).norm()
					+ (nextForcePointInPlane - nextAttachmentOriginInPlane).norm();

	return previousAttachmentPointOrigin(1)
			- (previousAttachmentPointOrigin(1) - nextAttachmentPointOrigin(1))
					* ((previousForcePointInPlane
							- previousAttachmentOriginInPlane).norm()
							+ angle * radius) / totalLength;
}
}
}
}
}
}
}
}
