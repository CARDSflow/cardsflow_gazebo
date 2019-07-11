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

#include "IActuator.h"

class Actuator: public IActuator
{
public:
	Actuator();

	virtual ~Actuator();

//	virtual void init(const QDomElement& element);

	double getAnchorResistance() const;

	void setAnchorResistance(double anchorResistance);

	double getHbridgeResistance() const;

	void setHbridgeResistance(double hbridgeResistance);

	double getAnchorInductance() const;

	void setAnchorInductance(double anchorInductance);

	double getBackEmfConstant() const;

	void setBackEmfConstant(double backEmfConstant);

	double getTorqueConstant() const;

	void setTorqueConstant(double torqueConstant);

	double getMomentOfInertiaMotor() const;

	void setMomentOfInertiaMotor(double momentOfInertiaMotor);

	double getMomentOfInertiaGearbox() const;

	void setMomentOfInertiaGearbox(double momentOfInertiaGearbox);

	double getGearboxEfficiency() const;

	void setGearboxEfficiency(double gearboxEfficiency);

	double getGearboxRatio() const;

	void setGearboxRatio(double gearboxRatio);

	double getSpindleRadius() const;

	void setSpindleRadius(double spindleRadius);

	void setVoltage(double voltage);

	double getVoltage() const;

	void setLoadTorque(double torque);

	double getAngularVelocity() const;

	double getLinearVelocity() const;

	double getCurrent() const;

	double getPosition() const;

	double getInitialPosition() const;

	double getRho() const;

	void step(const double time);

	void setTimeStep(double timeStep);

	double getTimeStep();

	void setIntegrator(Integrator integrator);

	Integrator getIntegrator();

	void printModelMatrices();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	double anchorResistance;

	double hbridgeResistance;

	double anchorInductance;

	double backEmfConstant;

	double torqueConstant;

	double momentOfInertiaMotor;

	double momentOfInertiaGearbox;

	double gearboxEfficiency;

	double steepness;

	double shift;

	double rhoDiff;

	double rho;

	double gearboxRatio;

	double spindleRadius;

	double position;

	double initialPosition;

	double timeStep;

	double correctedTimeStep;

	Eigen::Matrix2d A;

	Eigen::Matrix2d B;

	Eigen::Vector2d x;

	Eigen::Vector2d u;

	Eigen::Vector2d dx;

	Eigen::Vector2d k1;

	Eigen::Vector2d k2;

	Eigen::Vector2d k3;

	Eigen::Vector2d k4;

	Integrator integrator;

	unsigned int steps;

	void setA(unsigned int row, unsigned int column);

	void setB(unsigned int row, unsigned int column);
};


