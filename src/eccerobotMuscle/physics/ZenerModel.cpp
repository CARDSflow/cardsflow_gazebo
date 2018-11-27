/*
 *  Copyright (c) 2013, MYOROBOTICS consortium
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

#include <iostream>
#include <math.h>
#include "cardsflow_gazebo/eccerobotMuscle/physics/ZenerModel.h"


ZenerModel::ZenerModel() :
		E0(0), E1(0), mu1(0), y(0), strain(0.0), L0(0), A0(0), dt(1e-4), force(
				0.0), segment(0), velocity(0.0), firstUpdate(true), previousStrain(
				0.0)
{

}
ZenerModel::~ZenerModel()
{

}

//void ZenerModel::init(const QDomElement& element)
//{
//	if (element.hasAttribute("timeStep"))
//	{
//		dt = element.attribute("timeStep").toDouble();
//	}
//
//	if (element.hasAttribute("inSegment"))
//	{
//		segment = element.attribute("inSegment").toUInt();
//	}
//
//	QDomElement child;
//
//	child = element.firstChildElement("A0");
//
//	if (!child.isNull())
//	{
//		A0 = child.text().toDouble();
//	}
//	else
//	{
//		throw ParseException("no A0 element");
//	}
//
//	child = element.firstChildElement("L0");
//
//	if (!child.isNull())
//	{
//		L0 = child.text().toDouble();
//	}
//	else
//	{
//		throw ParseException("no L0 element");
//	}
//
//	child = element.firstChildElement("E0");
//
//	if (!child.isNull())
//	{
//		E0 = child.text().toDouble();
//	}
//	else
//	{
//		throw ParseException("no E0 element");
//	}
//
//	child = element.firstChildElement("E1");
//
//	if (!child.isNull())
//	{
//		E1 = child.text().toDouble();
//	}
//	else
//	{
//		throw ParseException("no E1 element");
//	}
//
//	child = element.firstChildElement("mu1");
//
//	if (!child.isNull())
//	{
//		mu1 = child.text().toDouble();
//	}
//	else
//	{
//		throw ParseException("no mu1 element");
//	}
//}

void ZenerModel::update(rl::math::Real length, rl::math::Real time)
{
	strain = length / L0 - 1;

	if (firstUpdate)
	{
		previousStrain = strain;
	}

	velocity = (strain * L0 - previousStrain * L0) / time;

	previousStrain = strain;

	if (strain < 0)
	{
		strain = 0.0;
	}

	steps = time / dt;

	dtCorrected = time / steps;

	for (int i = 0; i < steps; i++)
	{
		update2(strain, dtCorrected);
	}

	if (force < 0.0)
	{
		force = 0.0;
	}

	firstUpdate = false;
}

void ZenerModel::update2(rl::math::Real strain, rl::math::Real dt)
{
	k1 = E1 * E1 * strain / mu1 - E1 / mu1 * y;

	k2 = E1 * E1 * strain / mu1 - E1 / mu1 * (k1 * dt / 2.0 + y);

	k3 = E1 * E1 * strain / mu1 - E1 / mu1 * (k2 * dt / 2.0 + y);

	k4 = E1 * E1 * strain / mu1 - E1 / mu1 * (k3 * dt + y);

	y = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

	force = ((E0 + E1) * strain - y) * A0;
}

unsigned int ZenerModel::isInSegment()
{
	return segment;
}

rl::math::Real ZenerModel::getForce()
{
	return force;
}

rl::math::Real ZenerModel::getLength()
{
	return L0 * (strain + 1);
}

rl::math::Real ZenerModel::getVelocity()
{
	return velocity;
}

rl::math::Real ZenerModel::getRestingLength()
{
	return L0;
}

void ZenerModel::setRestingLength(rl::math::Real restingLength)
{
	L0 = restingLength;
}

void ZenerModel::setA0(rl::math::Real A0)
{
	this->A0 = A0;
}

rl::math::Real ZenerModel::getA0()
{
	return A0;
}

void ZenerModel::setE0(rl::math::Real E0)
{
	this->E0 = E0;
}

rl::math::Real ZenerModel::getE0()
{
	return E0;
}

void ZenerModel::setE1(rl::math::Real E1)
{
	this->E1 = E1;
}

rl::math::Real ZenerModel::getE1()
{
	return E1;
}

void ZenerModel::setMu1(rl::math::Real mu1)
{
	this->mu1 = mu1;
}

rl::math::Real ZenerModel::getMu1()
{
	return mu1;
}

