/*
 *  Copyright (c) 2012-2013, MYOROBOTICS consortium
 *  Author: Steffen Wittmeier
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

#include "SphericalPulley.h"
#include "AttachmentPoint.h"

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
AttachmentPoint::AttachmentPoint(sgal::physics::Body* body,
		const ::rl::math::Transform &frame, IMuscle* muscle) :
		body(body), frameLocal(frame), previous(0), next(0), muscle(muscle), uc(
				0.0), us(0.0), vs(1.0), fa(0.0), fb(0.0), previousSegmentKiteLineVelocity(
				0.0)
{
}

AttachmentPoint::~AttachmentPoint()
{
}

IAttachmentPoint::Type AttachmentPoint::getType() const
{
	return IAttachmentPoint::FixPoint;
}

void AttachmentPoint::updatePosition(rl::math::Real time)
{
	body->getFrame(bodyFrame);

	frameGlobal = bodyFrame * frameLocal;
}

void AttachmentPoint::updateForcePoints(rl::math::Real time)
		throw (sgal::Exception)
{

}

void AttachmentPoint::applyForce(rl::math::Real time,
		rl::math::Real previousSegmentKiteLineVelocity, bool beforeSEE)
				throw (sgal::Exception)
{
	this->previousSegmentKiteLineVelocity = previousSegmentKiteLineVelocity;

	if (previous && next)
	{
		A = (previous->getNextForcePointInGlobal()
				- getPreviousForcePointInGlobal()).normalized();

		B =
				(next->getPreviousForcePointInGlobal()
						- getNextForcePointInGlobal()).normalized();

		rl::math::Real dot = A.dot(B);

		if (dot > 1.0)
		{
			dot = 1.0;
		}
		else if (dot < -1.0)
		{
			dot = -1.0;
		}

		rl::math::Real alpha = M_PI - std::acos(dot);

		rl::math::Real absVel = std::abs(previousSegmentKiteLineVelocity);

		// adjust vs / x to make curve steeper or smoother
		rl::math::Real sigmoid = 1
				/ (1 + exp(-previousSegmentKiteLineVelocity / (vs / 5)));

		// adjust sigmoid to range [-1,1]
		rl::math::Real sign = 2 * (sigmoid - 0.5);

		// compute friction coefficient using simplified stribeck model
		rl::math::Real mu = (uc + (us - uc) * exp(-1.0 * absVel / vs)) * sign;

		if (beforeSEE)
		{
			fb = next->getPreviousForce();

			fa = fb * exp (- mu * alpha);
		}
		else
		{
			fa = previous->getNextForce();

			fb = fa * exp (mu * alpha);
		}

		Fa = A * fa;

		Fb = B * fb;

		body->applyForce(Fa,
				getPreviousForcePointInGlobal() - bodyFrame.translation());

		body->applyForce(Fb,
				getNextForcePointInGlobal() - bodyFrame.translation());
	}
	else if (next)
	{
		fb = next->getPreviousForce();

		B =
				(next->getPreviousForcePointInGlobal()
						- getNextForcePointInGlobal()).normalized();

		Fb = B * fb;

		body->applyForce(Fb,
				getNextForcePointInGlobal() - bodyFrame.translation());
	}
	else if (previous)
	{
		fa = previous->getNextForce();

		A = (previous->getNextForcePointInGlobal()
				- getPreviousForcePointInGlobal()).normalized();

		Fa = A * fa;

		body->applyForce(Fa,
				getPreviousForcePointInGlobal() - bodyFrame.translation());
	}
}

rl::math::Vector3 AttachmentPoint::getFrameInLocal() const
{
	return frameLocal.translation();
}

void AttachmentPoint::setFrameInLocal(const rl::math::Vector3& frameInLocal)
{
	this->frameLocal.translation() = frameInLocal;
}

rl::math::Vector3 AttachmentPoint::getFrameInGlobal() const
{
	return frameGlobal.translation();
}

rl::math::Transform AttachmentPoint::getTransformInGlobal() const
{
	return frameGlobal;
}

rl::math::Transform AttachmentPoint::getTransformInLocal() const
{
	return frameLocal;
}

sgal::physics::Body* AttachmentPoint::getBody() const
{
	return body;
}

rl::math::Real AttachmentPoint::getPreviousForce() const
{
	return fa;
}

void AttachmentPoint::setPreviousForce(rl::math::Real force)
{
	fa = force;
}

rl::math::Real AttachmentPoint::getNextForce() const
{
	return fb;
}

void AttachmentPoint::setNextForce(rl::math::Real force)
{
	fb = force;
}

void AttachmentPoint::setPreviousAttachmentPoint(IAttachmentPoint* previous)
{
	this->previous = previous;
}

IAttachmentPoint* AttachmentPoint::getPreviousAttachmentPoint() const
{
	return previous;
}

void AttachmentPoint::setNextAttachmentPoint(IAttachmentPoint* next)
{
	this->next = next;
}

IAttachmentPoint* AttachmentPoint::getNextAttachmentPoint() const
{
	return next;
}

rl::math::Vector3 AttachmentPoint::getPreviousForcePointInGlobal() const
{
	return frameGlobal.translation();
}

rl::math::Vector3 AttachmentPoint::getNextForcePointInGlobal() const
{
	return frameGlobal.translation();
}

rl::math::Real AttachmentPoint::getPreviousSegmentLength() const
{
	if (previous)
	{
		return (previous->getNextForcePointInGlobal()
				- getPreviousForcePointInGlobal()).norm();
	}

	return 0;
}

rl::math::Real AttachmentPoint::getPreviousSegmentKiteLineVelocity() const
{
	return previousSegmentKiteLineVelocity;
}

void AttachmentPoint::getTendonNodes(QVector< rl::math::Vector3 >& tendonNodes)
{
	tendonNodes.push_back(frameGlobal.translation());
}

void AttachmentPoint::setUc(rl::math::Real uc)
{
	this->uc = uc;
}

rl::math::Real AttachmentPoint::getUc()
{
	return uc;
}

void AttachmentPoint::setUs(rl::math::Real us)
{
	this->us = us;
}

rl::math::Real AttachmentPoint::getUs()
{
	return us;
}

void AttachmentPoint::setVs(rl::math::Real vs)
{
	this->vs = vs;
}
}
}
}
}
}
}
}
