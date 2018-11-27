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

#include "cardsflow_gazebo/eccerobotMuscle/physics/IActuator.h"

class Actuator: public IActuator
{
public:
	Actuator();

	virtual ~Actuator();

//	virtual void init(const QDomElement& element);

	rl::math::Real getAnchorResistance() const;

	void setAnchorResistance(rl::math::Real anchorResistance);

	rl::math::Real getHbridgeResistance() const;

	void setHbridgeResistance(rl::math::Real hbridgeResistance);

	rl::math::Real getAnchorInductance() const;

	void setAnchorInductance(rl::math::Real anchorInductance);

	rl::math::Real getBackEmfConstant() const;

	void setBackEmfConstant(rl::math::Real backEmfConstant);

	rl::math::Real getTorqueConstant() const;

	void setTorqueConstant(rl::math::Real torqueConstant);

	rl::math::Real getMomentOfInertiaMotor() const;

	void setMomentOfInertiaMotor(rl::math::Real momentOfInertiaMotor);

	rl::math::Real getMomentOfInertiaGearbox() const;

	void setMomentOfInertiaGearbox(rl::math::Real momentOfInertiaGearbox);

	rl::math::Real getGearboxEfficiency() const;

	void setGearboxEfficiency(rl::math::Real gearboxEfficiency);

	rl::math::Real getGearboxRatio() const;

	void setGearboxRatio(rl::math::Real gearboxRatio);

	rl::math::Real getSpindleRadius() const;

	void setSpindleRadius(rl::math::Real spindleRadius);

	void setVoltage(rl::math::Real voltage);

	rl::math::Real getVoltage() const;

	void setLoadTorque(rl::math::Real torque);

	rl::math::Real getAngularVelocity() const;

	rl::math::Real getLinearVelocity() const;

	rl::math::Real getCurrent() const;

	rl::math::Real getPosition() const;

	rl::math::Real getInitialPosition() const;

	rl::math::Real getRho() const;

	void step(const rl::math::Real time);

	void setTimeStep(rl::math::Real timeStep);

	rl::math::Real getTimeStep();

	void setIntegrator(Integrator integrator);

	Integrator getIntegrator();

	void printModelMatrices();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	rl::math::Real anchorResistance;

	rl::math::Real hbridgeResistance;

	rl::math::Real anchorInductance;

	rl::math::Real backEmfConstant;

	rl::math::Real torqueConstant;

	rl::math::Real momentOfInertiaMotor;

	rl::math::Real momentOfInertiaGearbox;

	rl::math::Real gearboxEfficiency;

	rl::math::Real steepness;

	rl::math::Real shift;

	rl::math::Real rhoDiff;

	rl::math::Real rho;

	rl::math::Real gearboxRatio;

	rl::math::Real spindleRadius;

	rl::math::Real position;

	rl::math::Real initialPosition;

	rl::math::Real timeStep;

	rl::math::Real correctedTimeStep;

	rl::math::Matrix22 A;

	rl::math::Matrix22 B;

	rl::math::Vector2 x;

	rl::math::Vector2 u;

	rl::math::Vector2 dx;

	rl::math::Vector2 k1;

	rl::math::Vector2 k2;

	rl::math::Vector2 k3;

	rl::math::Vector2 k4;

	Integrator integrator;

	unsigned int steps;

	void setA(unsigned int row, unsigned int column);

	void setB(unsigned int row, unsigned int column);
};

