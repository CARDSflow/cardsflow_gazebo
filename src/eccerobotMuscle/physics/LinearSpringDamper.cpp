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

#include "LinearSpringDamper.h"
#include "../ParseException.h"
#include <iostream>

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
LinearSpringDamper::LinearSpringDamper() :
		restingLength(0.0), springConstant(0.0), dampingConstant(0.0), asymmetricMode(
				false), previousExpansion(0.0), force(0.0), segment(0), velocity(
				0.0), firstUpdate(true)
{

}

LinearSpringDamper::~LinearSpringDamper()
{

}

void LinearSpringDamper::init(const QDomElement& element)
{
	if (element.hasAttribute("inSegment"))
	{
		segment = element.attribute("inSegment").toUInt();
	}

	QDomElement child;

	child = element.firstChildElement("springConstant");

	if (!child.isNull())
	{
		springConstant = child.text().toDouble();
	}
	else
	{
		throw ParseException("no springConstant element");
	}

	child = element.firstChildElement("restingLength");

	if (!child.isNull())
	{
		restingLength = child.text().toDouble();
	}
	else
	{
		throw ParseException("no restingLength element");
	}

	child = element.firstChildElement("dampingConstant");

	if (!child.isNull())
	{
		dampingConstant = child.text().toDouble();
	}
	else
	{
		throw ParseException("no dampingConstant element");
	}

	child = element.firstChildElement("asymetricMode");

	if (!child.isNull())
	{
		if (child.text() == QString("true"))
		{
			asymmetricMode = true;
		}
		else
		{
			asymmetricMode = false;
		}
	}
	else
	{
		throw ParseException("no asymetricMode element");
	}
}

void LinearSpringDamper::update(rl::math::Real length, rl::math::Real time)
{
	force = 0.0;

	double expansion = length - restingLength;

	if (firstUpdate)
	{
		previousExpansion = expansion;
	}

	velocity = (expansion - previousExpansion) / time;

	// if asymmetric mode is active set expansion and force to zero
	if (asymmetricMode && expansion > 0)
	{
		force = springConstant * expansion + dampingConstant * velocity;

		// if asymmetric mode is active it can happen that
		// the damping force is higher than the spring force and therefore
		// that the force becomes negative -> then it has to be set to 0
		if (asymmetricMode && (force <= 0))
		{
			force = 0.0;
		}
	}

	previousExpansion = expansion;

	firstUpdate = false;
}

rl::math::Real LinearSpringDamper::getForce()
{
	return force;
}

rl::math::Real LinearSpringDamper::getLength()
{
	return restingLength + previousExpansion;
}

rl::math::Real LinearSpringDamper::getRestingLength()
{
	return restingLength;
}

rl::math::Real LinearSpringDamper::getVelocity()
{
	return velocity;
}

void LinearSpringDamper::setRestingLength(rl::math::Real restingLength)
{
	this->restingLength = restingLength;
}

unsigned int LinearSpringDamper::isInSegment()
{
	return segment;
}

void LinearSpringDamper::setDampingConstant(rl::math::Real dampingConstant)
{
	this->dampingConstant = dampingConstant;
}

rl::math::Real LinearSpringDamper::getDampingConstant()
{
	return dampingConstant;
}

void LinearSpringDamper::setSpringConstant(rl::math::Real springConstant)
{
	this->springConstant = springConstant;
}

rl::math::Real LinearSpringDamper::getSpringConstant()
{
	return springConstant;
}

void LinearSpringDamper::useAsymmetricMode(bool asymmetricMode)
{
	this->asymmetricMode = asymmetricMode;
}

bool LinearSpringDamper::isAsymmetricMode()
{
	return asymmetricMode;
}
}
}
}
}
}
}
}
