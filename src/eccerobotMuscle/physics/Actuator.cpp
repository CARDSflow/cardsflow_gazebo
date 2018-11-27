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

#include <math.h>
#include <iostream>
#include <rl/math/Unit.h>

#include "cardsflow_gazebo/eccerobotMuscle/physics/Actuator.h"

Actuator::Actuator() :
		anchorResistance(0.0), anchorInductance(0.0), backEmfConstant(0.0), torqueConstant(
				0.0), momentOfInertiaMotor(0.0), momentOfInertiaGearbox(0.0), gearboxEfficiency(
				0.0), gearboxRatio(0.0), spindleRadius(0.0), integrator(
				RungeKutta4), timeStep(1e-4), position(0.0), rhoDiff(0.0), steepness(
				500), hbridgeResistance(0.0), initialPosition(0.0), shift(5)
{
	u.setZero();

	x.setZero();

	A.setZero();

	B.setZero();
}

Actuator::~Actuator()
{
}

//void Actuator::init(const QDomElement& element)
//{
//	QDomElement childElement;
//
//	if (element.hasAttribute("integrator"))
//	{
//		integrator = (Integrator) element.attribute("integrator").toInt();
//	}
//
//	if (element.hasAttribute("timeStep"))
//	{
//		timeStep = element.attribute("timeStep").toDouble();
//	}
//
//	childElement = element.firstChildElement("initialMotorPosition");
//
//	if (!childElement.isNull())
//	{
//		position = initialPosition = childElement.text().toDouble();
//	}
//	else
//	{
//		throw ParseException("physics: no initialMotorPosition element");
//	}
//
//	childElement = element.firstChildElement("anchorResistance");
//
//	if (!childElement.isNull())
//	{
//		this->setAnchorResistance(childElement.text().toDouble());
//	}
//	else
//	{
//		throw ParseException("physics: no anchorResistance element");
//	}
//
//	childElement = element.firstChildElement("hbridgeResistance");
//
//	if (!childElement.isNull())
//	{
//		this->setHbridgeResistance(childElement.text().toDouble());
//	}
//	else
//	{
//		throw ParseException("physics: no hbridgeResistance element");
//	}
//
//	childElement = element.firstChildElement("anchorInductance");
//
//	if (!childElement.isNull())
//	{
//		this->setAnchorInductance(childElement.text().toDouble());
//	}
//	else
//	{
//		throw ParseException("physics: no anchorInductance element");
//	}
//
//	childElement = element.firstChildElement("backEmfConstant");
//
//	if (!childElement.isNull())
//	{
//		this->setBackEmfConstant(childElement.text().toDouble());
//	}
//	else
//	{
//		throw ParseException("physics: no backEmfConstant element");
//	}
//
//	childElement = element.firstChildElement("torqueConstant");
//
//	if (!childElement.isNull())
//	{
//		this->setTorqueConstant(childElement.text().toDouble());
//	}
//	else
//	{
//		throw ParseException("physics: no torqueConstant element");
//	}
//
//	childElement = element.firstChildElement("momentOfInertiaMotor");
//
//	if (!childElement.isNull())
//	{
//		this->setMomentOfInertiaMotor(childElement.text().toDouble());
//	}
//	else
//	{
//		throw ParseException("physics: no momentOfInertiaMotor element");
//	}
//
//	childElement = element.firstChildElement("momentOfInertiaGearbox");
//
//	if (!childElement.isNull())
//	{
//		this->setMomentOfInertiaGearbox(childElement.text().toDouble());
//	}
//	else
//	{
//		throw ParseException("physics: no momentOfInertiaGearbox element");
//	}
//
//	childElement = element.firstChildElement("gearboxEfficiency");
//
//	if (!childElement.isNull())
//	{
//		this->setGearboxEfficiency(childElement.text().toDouble());
//	}
//	else
//	{
//		throw ParseException("physics: no gearboxEfficiency element");
//	}
//
//	childElement = element.firstChildElement("gearboxRatio");
//
//	if (!childElement.isNull())
//	{
//		this->setGearboxRatio(childElement.text().toDouble());
//	}
//	else
//	{
//		throw ParseException("physics: no gearboxRatio element");
//	}
//
//	childElement = element.firstChildElement("spindleRadius");
//
//	if (!childElement.isNull())
//	{
//		this->setSpindleRadius(childElement.text().toDouble());
//	}
//	else
//	{
//		throw ParseException("physics: no spindleRadius element");
//	}
//}

rl::math::Real Actuator::getAnchorResistance() const
{
	return anchorResistance;
}

void Actuator::setAnchorResistance(rl::math::Real anchorResistance)
{
	this->anchorResistance = anchorResistance;
	setA(0, 0);
}

rl::math::Real Actuator::getHbridgeResistance() const
{
	return hbridgeResistance;
}

void Actuator::setHbridgeResistance(rl::math::Real hbridgeResistance)
{
	this->hbridgeResistance = hbridgeResistance;
	setA(0, 0);
}

rl::math::Real Actuator::getAnchorInductance() const
{
	return anchorInductance;
}

void Actuator::setAnchorInductance(rl::math::Real anchorInductance)
{
	this->anchorInductance = anchorInductance;
	setA(0, 0);
	setA(0, 1);
	setB(0, 0);
}

rl::math::Real Actuator::getBackEmfConstant() const
{
	return backEmfConstant;
}

void Actuator::setBackEmfConstant(rl::math::Real backEmfConstant)
{
	this->backEmfConstant = backEmfConstant;
	setA(0, 1);
}

rl::math::Real Actuator::getTorqueConstant() const
{
	return torqueConstant;
}

void Actuator::setTorqueConstant(rl::math::Real torqueConstant)
{
	this->torqueConstant = torqueConstant;
	setA(1, 0);
}

rl::math::Real Actuator::getMomentOfInertiaMotor() const
{
	return momentOfInertiaMotor;
}

void Actuator::setMomentOfInertiaMotor(rl::math::Real momentOfInertiaMotor)
{
	this->momentOfInertiaMotor = momentOfInertiaMotor;
	setA(1, 0);
	setB(1, 1);
}

rl::math::Real Actuator::getMomentOfInertiaGearbox() const
{
	return momentOfInertiaGearbox;
}

void Actuator::setMomentOfInertiaGearbox(rl::math::Real momentOfInertiaGearbox)
{
	this->momentOfInertiaGearbox = momentOfInertiaGearbox;
	setA(1, 0);
	setB(1, 1);
}

rl::math::Real Actuator::getGearboxEfficiency() const
{
	return gearboxEfficiency;
}

void Actuator::setGearboxEfficiency(rl::math::Real gearboxEfficiency)
{
	this->gearboxEfficiency = gearboxEfficiency;
	rho = gearboxEfficiency;
	rhoDiff = 1.0 / gearboxEfficiency - gearboxEfficiency;
	setB(1, 1);
}

rl::math::Real Actuator::getGearboxRatio() const
{
	return gearboxRatio;
}

void Actuator::setGearboxRatio(rl::math::Real gearboxRatio)
{
	this->gearboxRatio = gearboxRatio;
	setA(0, 1);
	setA(1, 0);
	setB(1, 1);
}

rl::math::Real Actuator::getSpindleRadius() const
{
	return spindleRadius;
}

void Actuator::setSpindleRadius(rl::math::Real spindleRadius)
{
	this->spindleRadius = spindleRadius;
}

void Actuator::setVoltage(rl::math::Real voltage)
{
	u(0) = voltage;
}

rl::math::Real Actuator::getVoltage() const
{
	return u(0);
}

void Actuator::setLoadTorque(rl::math::Real torque)
{
	u(1) = torque;
}

rl::math::Real Actuator::getAngularVelocity() const
{
	return x(1);
}

rl::math::Real Actuator::getLinearVelocity() const
{
	return getAngularVelocity() * spindleRadius;
}

rl::math::Real Actuator::getCurrent() const
{
	return x(0);
}

rl::math::Real Actuator::getPosition() const
{
	return position;
}

rl::math::Real Actuator::getInitialPosition() const
{
	return initialPosition;
}

rl::math::Real Actuator::getRho() const
{
	return rho;
}

void Actuator::step(rl::math::Real time)
{
	steps = time / timeStep;

	correctedTimeStep = time / steps;

	for (int i = 0; i < steps; i++)
	{
		// adapt gearbox efficiency for forward and reverse mode using tanh
		// parameters steepness and shift can be used to adjust the smoothing
		rho = gearboxEfficiency
				+ rhoDiff * (std::tanh(-steepness * x(0) * x(1) - shift) + 1)
						/ 2;

		setB(1, 1);

		k1 = A * x + B * u;

		if (integrator == ForwardEuler)
		{
			x = x + (k1 * correctedTimeStep);
		}
		else if (integrator == RungeKutta4)
		{
			k2 = A * (k1 * correctedTimeStep / 2.0 + x) + B * u;

			k3 = A * (k2 * correctedTimeStep / 2.0 + x) + B * u;

			k4 = A * (k3 * correctedTimeStep + x) + B * u;

			x = x + (correctedTimeStep / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
		}

		position += getAngularVelocity() * correctedTimeStep
				* rl::math::RAD2DEG;
	}
}

void Actuator::setTimeStep(rl::math::Real timeStep)
{
	this->timeStep = timeStep;
}

rl::math::Real Actuator::getTimeStep()
{
	return timeStep;
}

void Actuator::setIntegrator(Actuator::Integrator integrator)
{
	this->integrator = integrator;
}

Actuator::Integrator Actuator::getIntegrator()
{
	return integrator;
}

void Actuator::printModelMatrices()
{
	std::cout << "A= " << A << std::endl;

	std::cout << "B= " << B << std::endl;
}

void Actuator::setA(unsigned int row, unsigned int column)
{
	if (row == 0 && column == 0)
	{
		if (anchorInductance != 0.0)
		{
			A(0, 0) = (-1.0 * (anchorResistance + hbridgeResistance)
					/ anchorInductance);
		}
	}
	else if (row == 0 && column == 1)
	{
		if (anchorInductance != 0.0)
		{
			A(0, 1) =
					(-1.0 * backEmfConstant * gearboxRatio / anchorInductance);
		}
	}
	else if (row == 1 && column == 0)
	{
		A(1, 0) = torqueConstant
				/ (gearboxRatio
						* (momentOfInertiaGearbox + momentOfInertiaMotor));
	}
	else if (row == 1 && column == 1)
	{
		A(1, 1) = 0.0;
	}
}

void Actuator::setB(unsigned int row, unsigned int column)
{
	if (row == 0 && column == 0)
	{
		if (anchorInductance != 0.0)
		{
			B(0, 0) = (1.0 / anchorInductance);
		}
	}
	else if (row == 0 && column == 1)
	{
		B(0, 1) = (0.0);
	}
	else if (row == 1 && column == 0)
	{
		B(1, 0) = (0.0);
	}
	else if (row == 1 && column == 1)
	{
		B(1, 1) = -1.0
				/ (rho * gearboxRatio * gearboxRatio
						* (momentOfInertiaGearbox + momentOfInertiaMotor));
	}
}

